//! `FakeFanctrl`: a scripted `FanctrlSource` (design doc §2.1 / Task 7,
//! fwloop.2) for controller/unit tests that need fw-fanctrl behavior
//! without a real socket. Records every command it receives (so a test can
//! assert the read-only invariant: only `Speed`/`All` were ever sent) and
//! replays a queue of scripted successes/failures, one per `poll()` call.

use std::collections::VecDeque;
use std::time::Instant;

use crate::fanctrl::client::{
    FanctrlError, FanctrlSnapshot, FanctrlSource, FanctrlView, Freshness, PrintCommand,
};

/// One scripted outcome for the next `poll()` call, regardless of which
/// `PrintCommand` it answers — the test itself controls that by choosing
/// which command it passes to `poll` alongside this entry, exactly as a
/// real caller would.
pub enum ScriptedOutcome {
    /// A successful `print all`: replaces the whole view (all fields),
    /// stamping both `observed_at` and `all_observed_at` with the `now`
    /// passed to `poll`. Use this even when scripting a response to a
    /// `Speed` poll is not the intent — pairing the wrong command with an
    /// `All` outcome is a test bug, not something this fake tries to guard
    /// against (see `FanctrlSource::poll`'s contract: the caller chooses
    /// the command it expects an answer to).
    All {
        strategy: String,
        active: bool,
        speed_pct: u8,
        temperature: f64,
        ma_temperature: f64,
        ma_interval: u32,
        curve: Vec<(f64, u8)>,
    },
    /// A successful `print speed`: updates only `speed_pct` and
    /// `observed_at` on the existing view. Dropped (a no-op success) if no
    /// view exists yet — same rule as `UnixFanctrlClient`, since a
    /// speed-only reading has nothing to attach itself to before the first
    /// `All`.
    Speed(u8),
    /// A failed poll.
    Fail(FanctrlError),
}

/// A scripted `FanctrlSource`. `script` a sequence of outcomes, then drive
/// it with `poll` in the same order a real controller would call it; each
/// `poll` consumes exactly one scripted outcome.
#[derive(Default)]
pub struct FakeFanctrl {
    log: Vec<PrintCommand>,
    view: Option<FanctrlView>,
    last_absent: bool,
    script: VecDeque<ScriptedOutcome>,
}

impl FakeFanctrl {
    pub fn new() -> Self {
        FakeFanctrl {
            log: Vec::new(),
            view: None,
            last_absent: false,
            script: VecDeque::new(),
        }
    }

    /// Queues one outcome for a future `poll()` call (FIFO).
    pub fn script(&mut self, outcome: ScriptedOutcome) {
        self.script.push_back(outcome);
    }

    /// Every command ever sent, in order. The read-only invariant this
    /// module exists to let a test assert: this must never contain
    /// anything but `Speed`/`All`, which is trivially true here (`log`
    /// entries come only from `poll`'s `cmd` parameter, itself a
    /// `PrintCommand`) but is exactly what a caller wants to assert against
    /// a fake standing in for the real, genuinely-write-capable-if-misused
    /// socket protocol.
    pub fn command_log(&self) -> &[PrintCommand] {
        &self.log
    }

    /// The last-known view. Inherent, not part of [`FanctrlSource`]: readers
    /// on another thread go through the published `FanctrlSnapshot` instead
    /// (see `sensors::poller`).
    pub fn view(&self) -> Option<&FanctrlView> {
        self.view.as_ref()
    }

    /// [`Freshness`] of `view()` as of `now` -- the same shared rule
    /// `UnixFanctrlClient` uses, so the two never drift apart.
    pub fn freshness(&self, now: Instant) -> Freshness {
        crate::fanctrl::client::compute_freshness(self.last_absent, self.view.as_ref(), now)
    }
}

impl FanctrlSource for FakeFanctrl {
    fn poll(&mut self, cmd: PrintCommand, now: Instant) -> Result<(), FanctrlError> {
        self.log.push(cmd);
        let outcome = self
            .script
            .pop_front()
            .expect("FakeFanctrl: poll() called with no scripted outcome queued");
        match outcome {
            ScriptedOutcome::Fail(e) => {
                self.last_absent = matches!(e, FanctrlError::Absent(_));
                Err(e)
            }
            ScriptedOutcome::All {
                strategy,
                active,
                speed_pct,
                temperature,
                ma_temperature,
                ma_interval,
                curve,
            } => {
                self.last_absent = false;
                self.view = Some(FanctrlView {
                    strategy,
                    active,
                    speed_pct,
                    temperature,
                    ma_temperature,
                    ma_interval,
                    curve,
                    observed_at: now,
                    all_observed_at: Some(now),
                });
                Ok(())
            }
            ScriptedOutcome::Speed(speed_pct) => {
                self.last_absent = false;
                if let Some(view) = &mut self.view {
                    view.speed_pct = speed_pct;
                    view.observed_at = now;
                }
                Ok(())
            }
        }
    }

    fn snapshot(&self) -> FanctrlSnapshot {
        FanctrlSnapshot {
            view: self.view.clone(),
            last_absent: self.last_absent,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn all_outcome(speed_pct: u8) -> ScriptedOutcome {
        ScriptedOutcome::All {
            strategy: "quiet16".to_string(),
            active: true,
            speed_pct,
            temperature: 75.0,
            ma_temperature: 75.0,
            ma_interval: 60,
            curve: vec![(0.0, 15), (95.0, 100)],
        }
    }

    #[test]
    fn records_every_command_and_nothing_else() {
        let mut fake = FakeFanctrl::new();
        fake.script(all_outcome(31));
        fake.script(ScriptedOutcome::Speed(32));
        fake.script(ScriptedOutcome::Fail(FanctrlError::Timeout));

        let t0 = Instant::now();
        fake.poll(PrintCommand::All, t0).unwrap();
        fake.poll(PrintCommand::Speed, t0 + Duration::from_secs(5))
            .unwrap();
        assert!(
            fake.poll(PrintCommand::Speed, t0 + Duration::from_secs(10))
                .is_err()
        );

        assert_eq!(
            fake.command_log(),
            &[PrintCommand::All, PrintCommand::Speed, PrintCommand::Speed]
        );
        // Nothing but Speed/All ever appears — the closed PrintCommand enum
        // already guarantees this at compile time, but the assertion below
        // is the shape a consuming test actually writes (design doc §2.1 /
        // Global Constraints: "any test that observes a command log
        // asserts nothing else was ever sent").
        assert!(
            fake.command_log()
                .iter()
                .all(|c| matches!(c, PrintCommand::Speed | PrintCommand::All))
        );
    }

    #[test]
    fn a_scripted_speed_success_bumps_observed_at_only() {
        let mut fake = FakeFanctrl::new();
        fake.script(all_outcome(31));
        fake.script(ScriptedOutcome::Speed(40));
        let t0 = Instant::now();
        let t1 = t0 + Duration::from_secs(5);
        fake.poll(PrintCommand::All, t0).unwrap();
        fake.poll(PrintCommand::Speed, t1).unwrap();
        let view = fake.view().unwrap();
        assert_eq!(view.speed_pct, 40);
        assert_eq!(view.observed_at, t1);
        assert_eq!(view.all_observed_at, Some(t0));
    }

    #[test]
    fn a_scripted_speed_success_before_any_all_is_dropped() {
        let mut fake = FakeFanctrl::new();
        fake.script(ScriptedOutcome::Speed(40));
        fake.poll(PrintCommand::Speed, Instant::now()).unwrap();
        assert!(fake.view().is_none());
    }

    #[test]
    #[should_panic(expected = "no scripted outcome queued")]
    fn polling_past_the_end_of_the_script_panics() {
        let mut fake = FakeFanctrl::new();
        let _ = fake.poll(PrintCommand::Speed, Instant::now());
    }

    #[test]
    fn scripted_failures_drive_freshness_the_same_way_as_the_real_client() {
        let mut fake = FakeFanctrl::new();
        fake.script(all_outcome(31));
        fake.script(ScriptedOutcome::Fail(FanctrlError::Absent(
            "ENOENT".to_string(),
        )));
        let t0 = Instant::now();
        fake.poll(PrintCommand::All, t0).unwrap();
        assert_eq!(fake.freshness(t0), Freshness::Fresh);
        let _ = fake.poll(PrintCommand::Speed, t0);
        assert_eq!(fake.freshness(t0), Freshness::Absent);
    }
}

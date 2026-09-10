//! Two background pollers whose I/O must never reach the sampler's 1 Hz tick
//! or the `print speed` staleness rule that governs Mode A (design doc §3.4):
//!
//! - [`FanctrlPoller`] runs `print speed` every 5 s and `print all` every
//!   30 s (never faster) against a [`FanctrlSource`] it **owns outright**,
//!   on its own thread. The socket round trip therefore holds no lock the
//!   sampler can ever contend on; after each attempt the poller publishes a
//!   detached [`FanctrlSnapshot`] into [`SharedFanctrl`], a tiny mutex whose
//!   only critical sections are one assignment (poller side) and one clone
//!   (sampler side). Neither side ever holds it across I/O, so a slow or
//!   hung socket round trip can never stall a `Sample`. (Before roast PR-1
//!   finding 1 the source itself lived behind that mutex and `poll` ran
//!   under it, which made this paragraph's invariant false: a 3 s read
//!   timeout blocked the 1 Hz tick for 3 s.)
//! - [`spawn_nvme_poller`] reads the NVMe composite temperature every 30 s on
//!   a thread of its own -- **neither** the sampler tick **nor**
//!   `FanctrlPoller`. A SMART admin read can block for the kernel's 60 s
//!   `admin_timeout`; on the sampler that would stall the control loop, and
//!   on `FanctrlPoller` it would fake a socket outage through the 15 s
//!   `print speed` staleness rule and drop the loop out of Mode A. It
//!   publishes a stamped last-good value; [`read_nvme`] turns a missing or
//!   stale one into `None`.
//!
//! Every cadence and staleness decision on this path is computed from
//! monotonic `Instant`s alone -- never a wall clock.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::fanctrl::client::{FanctrlSnapshot, FanctrlSource, PrintCommand};
use crate::sensors::sampler::sleep_unless_shutdown;
use crate::sync_util::lock;

/// `print speed` cadence -- never faster (design doc §2.1/§3.4).
pub const SPEED_PERIOD: Duration = Duration::from_secs(5);
/// `print all` cadence -- never faster.
pub const ALL_PERIOD: Duration = Duration::from_secs(30);
/// How often the poller thread wakes to check whether either cadence above
/// is due, and to poll the shutdown flag -- same granularity as the
/// sampler's own `SHUTDOWN_POLL`.
const POLLER_TICK: Duration = Duration::from_millis(250);

/// The published side of the fw-fanctrl poll: a detached
/// [`FanctrlSnapshot`], written by the poller thread and read by the sampler
/// tick. **Deliberately not the `FanctrlSource` itself** -- see the module
/// doc: this mutex is only ever held for one assignment or one clone, never
/// across socket I/O.
pub type SharedFanctrl = Arc<Mutex<FanctrlSnapshot>>;

/// A fresh, empty [`SharedFanctrl`] (no view yet, not absent) -- the state
/// before the poller's first attempt lands.
pub fn shared_fanctrl() -> SharedFanctrl {
    Arc::new(Mutex::new(FanctrlSnapshot::default()))
}

/// Publishes `source`'s current snapshot into `shared`. The only writer in
/// production is [`FanctrlPoller::tick`]; tests that drive a source by hand
/// (rather than through the poller thread) use this to make its state
/// visible to a `Sampler` the same way.
pub fn publish(shared: &SharedFanctrl, source: &dyn FanctrlSource) {
    *lock(shared) = source.snapshot();
}

/// Runs `print speed`/`print all` at their fixed cadences on its own thread,
/// against a [`FanctrlSource`] it owns exclusively, publishing the resulting
/// snapshot into a [`SharedFanctrl`] after each attempt.
pub struct FanctrlPoller {
    /// Owned outright, never shared: nothing else can contend on it, so the
    /// blocking round trip below cannot stall any other thread.
    source: Box<dyn FanctrlSource + Send>,
    snapshot: SharedFanctrl,
    next_speed_due: Instant,
    next_all_due: Instant,
}

impl FanctrlPoller {
    /// `start` is the instant both cadences are first due, so the very first
    /// `tick`/thread iteration sends both `Speed` and `All`.
    pub fn new(
        source: Box<dyn FanctrlSource + Send>,
        snapshot: SharedFanctrl,
        start: Instant,
    ) -> Self {
        Self {
            source,
            snapshot,
            next_speed_due: start,
            next_all_due: start,
        }
    }

    /// Sends whichever command(s) are due as of `now`, never faster than
    /// their own period, then publishes the resulting snapshot; a poll
    /// failure is swallowed here (the published snapshot's own `freshness()`
    /// is how a caller learns about it). This is the testable core:
    /// `spawn`'s real-time loop calls it with `Instant::now()`, and tests
    /// call it directly with synthetic instants stepped in 1 s increments to
    /// script a run without any real sleeping.
    ///
    /// The `poll` calls run outside every lock; only `publish` takes one.
    pub fn tick(&mut self, now: Instant) {
        if now >= self.next_all_due {
            let _ = self.source.poll(PrintCommand::All, now);
            self.next_all_due += ALL_PERIOD;
            publish(&self.snapshot, self.source.as_ref());
        }
        if now >= self.next_speed_due {
            let _ = self.source.poll(PrintCommand::Speed, now);
            self.next_speed_due += SPEED_PERIOD;
            publish(&self.snapshot, self.source.as_ref());
        }
    }

    /// Runs `tick` on a dedicated thread until `shutdown` flips, waking at
    /// least every [`POLLER_TICK`] so shutdown latency is bounded (matching
    /// the sampler's own shutdown responsiveness).
    pub fn spawn(mut self, shutdown: Arc<AtomicBool>) -> std::thread::JoinHandle<()> {
        std::thread::Builder::new()
            .name("fanctrl-poller".into())
            .spawn(move || {
                while !shutdown.load(Ordering::Relaxed) {
                    self.tick(Instant::now());
                    sleep_unless_shutdown(POLLER_TICK, &shutdown);
                }
                tracing::debug!("fanctrl-poller: shutdown flag set, exiting");
            })
            .expect("failed to spawn fanctrl-poller thread")
    }
}

/// NVMe temperature poll cadence -- its own thread, never the sampler tick or
/// `FanctrlPoller` (see module doc).
pub const NVME_PERIOD: Duration = Duration::from_secs(30);
/// A last-good NVMe reading older than this (or none at all) surfaces as
/// `None` on `Sample.nvme_temp_c` -- 3x its own cadence, the same ratio
/// `fanctrl::client`'s `ALL_STALE_AFTER` (90 s) uses over `print all`'s own
/// 30 s poll.
pub const NVME_STALE_AFTER: Duration = Duration::from_secs(90);

/// Last-good NVMe composite temperature plus the monotonic instant it was
/// read, shared between the NVMe poller thread (writer) and the sampler tick
/// (reader).
pub type SharedNvme = Arc<Mutex<Option<(f64, Instant)>>>;

/// The sampler tick's side of [`SharedNvme`]: a stale or never-populated
/// last-good value reads as `None`. `now` is caller-supplied (the sampler's
/// own monotonic `t_mono`-derived instant), never read internally, so this
/// is deterministic and never touches a wall clock.
pub fn read_nvme(cache: &SharedNvme, now: Instant) -> Option<f64> {
    let last = *lock(cache);
    last.and_then(|(temp, stamp)| {
        if now.saturating_duration_since(stamp) < NVME_STALE_AFTER {
            Some(temp)
        } else {
            None
        }
    })
}

/// Runs `read` at [`NVME_PERIOD`] on a dedicated thread, publishing every
/// `Some` result (stamped with the completion instant, not the call-start
/// instant) into `cache`. `read` may block far longer than `NVME_PERIOD` --
/// the whole reason this has its own thread, separate from the sampler and
/// `FanctrlPoller` -- a still-in-flight call simply delays the *next*
/// publish, and delays this thread noticing `shutdown` until the blocking
/// call itself returns (unavoidable: there is no way to cancel a blocked
/// sysfs read from the outside in std alone, mirrored by
/// `fanctrl::client::connect_with_timeout`'s own comment on the same
/// limitation).
pub fn spawn_nvme_poller<F>(
    mut read: F,
    cache: SharedNvme,
    shutdown: Arc<AtomicBool>,
) -> std::thread::JoinHandle<()>
where
    F: FnMut() -> Option<f64> + Send + 'static,
{
    std::thread::Builder::new()
        .name("nvme-poller".into())
        .spawn(move || {
            while !shutdown.load(Ordering::Relaxed) {
                if let Some(temp) = read() {
                    *lock(&cache) = Some((temp, Instant::now()));
                }
                sleep_unless_shutdown(NVME_PERIOD, &shutdown);
            }
            tracing::debug!("nvme-poller: shutdown flag set, exiting");
        })
        .expect("failed to spawn nvme-poller thread")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fanctrl::client::{FanctrlError, Freshness};
    use crate::test_support::fakes::{FakeFanctrl, ScriptedOutcome};

    fn all_success() -> ScriptedOutcome {
        ScriptedOutcome::All {
            strategy: "quiet16".to_string(),
            active: true,
            speed_pct: 31,
            temperature: 75.0,
            ma_temperature: 75.0,
            ma_interval: 60,
            curve: vec![(0.0, 15), (95.0, 100)],
        }
    }

    fn boxed(source: impl FanctrlSource + Send + 'static) -> Box<dyn FanctrlSource + Send> {
        Box::new(source) as Box<dyn FanctrlSource + Send>
    }

    // --- Step 2: cadence -------------------------------------------------

    /// Records every command it receives into a caller-visible log (kept as
    /// a second `Arc` clone, separate from the boxed `SharedFanctrl` handle
    /// the poller drives) -- `dyn FanctrlSource` has no `command_log()`
    /// accessor of its own (only `FakeFanctrl`, the concrete type, does), so
    /// a cadence test needs its own tiny recorder rather than trying to
    /// inspect the trait object after boxing.
    struct RecordingSource {
        log: Arc<Mutex<Vec<PrintCommand>>>,
    }

    impl FanctrlSource for RecordingSource {
        fn poll(&mut self, cmd: PrintCommand, _now: Instant) -> Result<(), FanctrlError> {
            self.log.lock().unwrap().push(cmd);
            Ok(())
        }
        fn snapshot(&self) -> FanctrlSnapshot {
            FanctrlSnapshot::default()
        }
    }

    #[test]
    fn speed_every_5s_and_all_every_30s_over_a_60s_scripted_run() {
        let log = Arc::new(Mutex::new(Vec::new()));
        let source = boxed(RecordingSource {
            log: Arc::clone(&log),
        });
        let mut poller = FanctrlPoller::new(source, shared_fanctrl(), Instant::now());

        // A "60 s scripted run": 60 ticks one simulated second apart, driven
        // through synthetic instants rather than real sleeping (mirrors
        // `Sampler::sample_at`'s own injectable-time test pattern).
        let t0 = Instant::now();
        for t in 0..60u64 {
            poller.tick(t0 + Duration::from_secs(t));
        }

        let log = log.lock().unwrap();
        let speed_count = log.iter().filter(|c| **c == PrintCommand::Speed).count();
        let all_count = log.iter().filter(|c| **c == PrintCommand::All).count();
        // Speed is due at every multiple of 5 in [0, 60): t = 0, 5, .., 55
        // -> 12 occurrences, exactly on cadence (this fixed-schedule `tick`
        // never drifts, so "±1 tick" from the acceptance criterion is
        // trivially satisfied -- there is no jitter to tolerate here).
        assert_eq!(speed_count, 12, "log: {log:?}");
        // All is due at every multiple of 30 in [0, 60): t = 0, 30 -> twice.
        assert_eq!(all_count, 2, "log: {log:?}");
        // At t=0 and t=30 both are due; `tick` sends All before Speed.
        assert_eq!(&log[0..2], &[PrintCommand::All, PrintCommand::Speed]);
    }

    // --- Step 3: freshness driven by the poller's own cadence ------------

    #[test]
    fn freshness_stale_after_90s_of_all_failures_with_speed_kept_fresh() {
        // Seed a view at t=0 (All succeeds), then All fails on every later
        // due tick (30, 60, 90) while Speed succeeds on every one of its own
        // due ticks -- isolating the 90 s `print all` window from the 15 s
        // `print speed` one (which stays fresh throughout).
        let mut fake = FakeFanctrl::new();
        for t in (0..=90).step_by(5) {
            if t % 30 == 0 {
                fake.script(if t == 0 {
                    all_success()
                } else {
                    ScriptedOutcome::Fail(FanctrlError::Timeout)
                });
            }
            fake.script(ScriptedOutcome::Speed(31));
        }
        let snapshot = shared_fanctrl();
        let mut poller = FanctrlPoller::new(boxed(fake), Arc::clone(&snapshot), Instant::now());

        let t0 = Instant::now();
        for t in (0..=90).step_by(5) {
            poller.tick(t0 + Duration::from_secs(t));
        }

        let src = lock(&snapshot).clone();
        assert_eq!(
            src.freshness(t0 + Duration::from_secs(89)),
            Freshness::Fresh,
            "89s since the last successful All: still inside the 90s window"
        );
        assert_eq!(
            src.freshness(t0 + Duration::from_secs(90)),
            Freshness::Stale,
            "90s since the last successful All: the window has elapsed"
        );
    }

    #[test]
    fn freshness_stale_after_15s_of_speed_failures_with_all_kept_fresh_by_the_seed() {
        // Seed a view at t=0 (both All and Speed succeed), then every later
        // poll (Speed at 5/10/15/20; no All is due again this early, its
        // next due tick is 30) fails -- isolating the 15 s `print speed`
        // window: `all_observed_at` stays pinned at 0, well inside its own
        // 90 s window throughout, so only the speed window can be what trips
        // Stale here.
        let mut fake = FakeFanctrl::new();
        fake.script(all_success()); // t=0 All
        fake.script(ScriptedOutcome::Speed(31)); // t=0 Speed
        for _ in (5..=20).step_by(5) {
            fake.script(ScriptedOutcome::Fail(FanctrlError::Timeout)); // Speed fails
        }
        let snapshot = shared_fanctrl();
        let mut poller = FanctrlPoller::new(boxed(fake), Arc::clone(&snapshot), Instant::now());

        let t0 = Instant::now();
        for t in (0..=20).step_by(5) {
            poller.tick(t0 + Duration::from_secs(t));
        }

        let src = lock(&snapshot).clone();
        assert_eq!(
            src.freshness(t0 + Duration::from_secs(14)),
            Freshness::Fresh,
            "14s since the last successful Speed (or All): still inside the 15s window"
        );
        assert_eq!(
            src.freshness(t0 + Duration::from_secs(15)),
            Freshness::Stale,
            "15s since the last successful Speed (or All): the window has elapsed"
        );
    }

    // --- Step 5: NVMe last-good cache -------------------------------------

    #[test]
    fn nvme_reading_flattens_to_none_when_never_populated_or_stale() {
        let cache: SharedNvme = Arc::new(Mutex::new(None));
        let t0 = Instant::now();
        assert_eq!(read_nvme(&cache, t0), None, "never populated");

        *lock(&cache) = Some((55.0, t0));
        assert_eq!(
            read_nvme(&cache, t0 + Duration::from_secs(89)),
            Some(55.0),
            "89s old: still inside the 90s staleness window"
        );
        assert_eq!(
            read_nvme(&cache, t0 + Duration::from_secs(90)),
            None,
            "90s old: the staleness window has elapsed"
        );
    }

    #[test]
    fn nvme_poller_publishes_a_stamped_reading_that_read_nvme_then_reports() {
        let cache: SharedNvme = Arc::new(Mutex::new(None));
        let shutdown = Arc::new(AtomicBool::new(false));
        let handle = spawn_nvme_poller(
            move || Some(42.0),
            Arc::clone(&cache),
            Arc::clone(&shutdown),
        );

        // Bounded wait for the thread's first (immediate) read to land --
        // avoids a fixed sleep that could be flaky under load.
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            if read_nvme(&cache, Instant::now()).is_some() {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "nvme poller never published a reading"
            );
            std::thread::sleep(Duration::from_millis(5));
        }
        assert_eq!(read_nvme(&cache, Instant::now()), Some(42.0));

        shutdown.store(true, Ordering::Relaxed);
        handle.join().expect("nvme poller thread should not panic");
    }
}

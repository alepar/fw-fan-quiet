//! GPU clock→watts calibration sweep (design doc §4.1): with the user running
//! a saturating GPU load, lock the SM clock at each step, verify the GPU is
//! actually pinned there, wait for the power reading to settle, record the
//! steady-state watts into a [`ClockWattsLut`].
//!
//! Sample-driven state machine, same pattern as the controller core: no
//! wall-clock sleeps, every `on_sample` returns [`SweepEffect`]s describing
//! what happened, so the whole sweep is unit-testable with synthetic samples.
//! The calibration runner (Task 22) is the shell that maps effects to
//! actuator commands and UI progress.

use std::collections::VecDeque;

use crate::calib::steady::{is_steady, tail_mean};
use crate::control::lut::ClockWattsLut;
use crate::types::Sample;

/// Clocks to sweep, descending from near-max to the low end: 10 steps about
/// 210 MHz apart. Descending order keeps the GPU load saturating from the
/// start (the top clock is where an unpinned GPU is most obvious).
// TODO(task-22): consumed by the calibration runner; dead until then.
#[allow(dead_code)]
pub const SWEEP_CLOCKS: [u32; 10] = [3090, 2880, 2670, 2460, 2250, 2040, 1830, 1620, 1410, 1200];

/// Tail-window length for GPU power settling. GPU power responds to a clock
/// lock much faster than fan RPM responds to heat, so this is shorter than
/// `steady::STEADY_N` (15 s vs 20 s at the 1 Hz sample rate).
#[allow(dead_code)]
pub const STEADY_N_GPU_W: usize = 15;

/// Max-min spread (watts) the tail window may have and still count as steady.
#[allow(dead_code)]
pub const GPU_W_TOLERANCE: f64 = 2.0;

/// Consecutive pinned samples required before we trust the clock lock took.
const PIN_STREAK: usize = 3;

/// Emit `NeedsLoad` after every this many consecutive unpinned samples (the
/// UI nags "start a GPU-heavy load" once per period, not once per sample).
const NEEDS_LOAD_EVERY: usize = 10;

/// Utilization above which the GPU counts as loaded.
const PIN_UTIL_MIN_PCT: f64 = 90.0;

/// Max |measured SM clock - commanded lock| (MHz) that still counts as pinned.
const PIN_CLOCK_TOLERANCE_MHZ: f64 = 30.0;

/// Watts-window cap. Settling detection normally bounds the window at
/// `STEADY_N_GPU_W`; the cap only prevents unbounded growth when a point
/// never settles. A never-steady point stalls the sweep forever — the UI's
/// Esc-abort (Task 22) is the way out, by design (better than recording a
/// bogus point).
const WATTS_WINDOW_CAP: usize = 60;

/// Where the sweep is for the current clock step (exposed for UI progress).
#[derive(Debug, Clone, PartialEq, Eq)]
// TODO(task-22): consumed by the calibration runner; dead until then.
#[allow(dead_code)]
pub enum SweepState {
    /// Waiting for the GPU to be loaded and locked at `clock`.
    WaitPinned { clock: u32 },
    /// Pinned; accumulating watts until the reading is steady.
    Settling { clock: u32 },
    /// All clocks recorded; `Finished` was emitted.
    Done,
}

/// What one `start`/`on_sample` call did — mapped to actuation/UI by the
/// runner shell (Task 22), asserted on directly in tests.
#[derive(Debug, Clone, PartialEq)]
// TODO(task-22): consumed by the calibration runner; dead until then.
#[allow(dead_code)]
pub enum SweepEffect {
    /// Lock the GPU SM clock at this many MHz.
    CommandClock(u32),
    /// GPU is not pinned; the UI should tell the user to start a GPU-heavy load.
    NeedsLoad,
    /// Steady-state point recorded.
    RecordPoint { mhz: u32, watts: f64 },
    /// Sweep complete; here is the built LUT.
    Finished(ClockWattsLut),
}

/// The sweep state machine. Drive it with `start()` once, then `on_sample`
/// for every 1 Hz sample.
// TODO(task-22): consumed by the calibration runner; dead until then.
#[allow(dead_code)]
pub struct LutSweep {
    /// Index into `SWEEP_CLOCKS` of the point being measured (== completed
    /// points; equals `SWEEP_CLOCKS.len()` when Done).
    idx: usize,
    state: SweepState,
    pinned_streak: usize,
    unpinned_streak: usize,
    /// gpu_w of valid samples while Settling (capped at `WATTS_WINDOW_CAP`).
    watts_window: VecDeque<f64>,
    lut: ClockWattsLut,
}

// TODO(task-22): consumed by the calibration runner; dead until then.
#[allow(dead_code)]
impl LutSweep {
    /// Positioned at the first clock; emits nothing until `start()`.
    pub fn new() -> Self {
        Self {
            idx: 0,
            state: SweepState::WaitPinned {
                clock: SWEEP_CLOCKS[0],
            },
            pinned_streak: 0,
            unpinned_streak: 0,
            watts_window: VecDeque::new(),
            lut: ClockWattsLut::new(),
        }
    }

    /// Begin the sweep: command the first clock lock.
    pub fn start(&mut self) -> Vec<SweepEffect> {
        vec![SweepEffect::CommandClock(SWEEP_CLOCKS[self.idx])]
    }

    /// True when the GPU is demonstrably loaded and locked at `clock`. An
    /// invalid GPU sample (power or clock reading missing) never counts as
    /// pinned: a sensor outage must not let the sweep proceed on phantom
    /// readings — and the check must not silently depend on the sampler's
    /// 0.0-sentinel flattening of missing values (`gpu_mhz_valid` is checked
    /// explicitly, not via a 0.0 clock failing the tolerance test).
    fn is_pinned(s: &Sample, clock: u32) -> bool {
        s.gpu_w_valid
            && s.gpu_mhz_valid
            && s.gpu_util_pct > PIN_UTIL_MIN_PCT
            && (s.gpu_sm_mhz - f64::from(clock)).abs() < PIN_CLOCK_TOLERANCE_MHZ
    }

    /// Consume one 1 Hz sample; returns what happened.
    pub fn on_sample(&mut self, s: &Sample) -> Vec<SweepEffect> {
        let mut effects = Vec::new();
        match self.state {
            SweepState::WaitPinned { clock } => {
                if Self::is_pinned(s, clock) {
                    self.unpinned_streak = 0;
                    self.pinned_streak += 1;
                    if self.pinned_streak >= PIN_STREAK {
                        self.pinned_streak = 0;
                        self.watts_window.clear();
                        self.state = SweepState::Settling { clock };
                    }
                } else {
                    self.pinned_streak = 0;
                    self.unpinned_streak += 1;
                    if self.unpinned_streak.is_multiple_of(NEEDS_LOAD_EVERY) {
                        effects.push(SweepEffect::NeedsLoad);
                    }
                }
            }
            SweepState::Settling { clock } => {
                // Re-check pinned-ness on every settling sample: if the GPU
                // load dies mid-settle, idle power is *very* flat, and
                // without this check 15 flat idle samples would record a
                // bogus point (e.g. 3090 MHz -> 15 W) and silently poison
                // the LUT. Any unpinned (or invalid) sample discards the
                // window and falls back to waiting; the NeedsLoad nag
                // re-arms with this sample as the first unpinned one.
                if !Self::is_pinned(s, clock) {
                    self.watts_window.clear();
                    self.pinned_streak = 0;
                    self.unpinned_streak = 1;
                    self.state = SweepState::WaitPinned { clock };
                    return effects;
                }
                if self.watts_window.len() == WATTS_WINDOW_CAP {
                    self.watts_window.pop_front();
                }
                self.watts_window.push_back(s.gpu_w);
                let window = self.watts_window.make_contiguous();
                if is_steady(window, STEADY_N_GPU_W, GPU_W_TOLERANCE) {
                    let watts = tail_mean(window, STEADY_N_GPU_W)
                        .expect("is_steady guarantees a full, NaN-free tail");
                    effects.push(SweepEffect::RecordPoint { mhz: clock, watts });
                    self.lut.insert(clock, watts);
                    self.idx += 1;
                    match SWEEP_CLOCKS.get(self.idx) {
                        Some(&next) => {
                            self.unpinned_streak = 0;
                            self.state = SweepState::WaitPinned { clock: next };
                            effects.push(SweepEffect::CommandClock(next));
                        }
                        None => {
                            self.state = SweepState::Done;
                            effects.push(SweepEffect::Finished(self.lut.clone()));
                        }
                    }
                }
            }
            SweepState::Done => {}
        }
        effects
    }

    /// Current state, for UI progress display.
    pub fn state(&self) -> &SweepState {
        &self.state
    }

    /// (completed points, total points).
    pub fn progress(&self) -> (usize, usize) {
        (self.idx, SWEEP_CLOCKS.len())
    }
}

impl Default for LutSweep {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A valid sample pinned at `clock` drawing `watts`.
    fn pinned(clock: u32, watts: f64) -> Sample {
        Sample {
            gpu_util_pct: 99.0,
            gpu_sm_mhz: f64::from(clock),
            gpu_w: watts,
            gpu_w_valid: true,
            gpu_mhz_valid: true,
            ..Sample::default()
        }
    }

    /// A valid sample with the GPU idle (not pinned, low util).
    fn idle() -> Sample {
        Sample {
            gpu_util_pct: 5.0,
            gpu_sm_mhz: 300.0,
            gpu_w: 15.0,
            gpu_w_valid: true,
            gpu_mhz_valid: true,
            ..Sample::default()
        }
    }

    fn record_points(effects: &[SweepEffect]) -> Vec<(u32, f64)> {
        effects
            .iter()
            .filter_map(|e| match e {
                SweepEffect::RecordPoint { mhz, watts } => Some((*mhz, *watts)),
                _ => None,
            })
            .collect()
    }

    fn commanded_clocks(effects: &[SweepEffect]) -> Vec<u32> {
        effects
            .iter()
            .filter_map(|e| match e {
                SweepEffect::CommandClock(mhz) => Some(*mhz),
                _ => None,
            })
            .collect()
    }

    /// Drive one full clock step: 3 pinned samples to pass WaitPinned, then
    /// steady watts until it records. Returns all effects emitted.
    fn drive_step(sweep: &mut LutSweep, clock: u32, watts: f64) -> Vec<SweepEffect> {
        let mut effects = Vec::new();
        // Pin phase: exactly PIN_STREAK samples flips to Settling.
        for _ in 0..PIN_STREAK {
            effects.extend(sweep.on_sample(&pinned(clock, watts)));
        }
        assert_eq!(*sweep.state(), SweepState::Settling { clock });
        // Settle phase: STEADY_N_GPU_W flat samples make it steady.
        for _ in 0..STEADY_N_GPU_W {
            effects.extend(sweep.on_sample(&pinned(clock, watts)));
        }
        effects
    }

    #[test]
    fn start_commands_first_clock() {
        let mut sweep = LutSweep::new();
        assert_eq!(*sweep.state(), SweepState::WaitPinned { clock: 3090 });
        assert_eq!(sweep.progress(), (0, 10));
        let effects = sweep.start();
        assert_eq!(effects, vec![SweepEffect::CommandClock(3090)]);
    }

    #[test]
    fn full_sweep_records_all_points_and_finishes() {
        let mut sweep = LutSweep::new();
        let mut all = sweep.start();
        // Watts roughly proportional to clock, distinct per step.
        let watts_at = |clock: u32| f64::from(clock) / 30.0;
        for (i, &clock) in SWEEP_CLOCKS.iter().enumerate() {
            assert_eq!(sweep.progress(), (i, 10));
            all.extend(drive_step(&mut sweep, clock, watts_at(clock)));
            assert_eq!(sweep.progress(), (i + 1, 10));
        }
        assert_eq!(*sweep.state(), SweepState::Done);

        // One RecordPoint per clock, with the expected (flat-window) watts.
        let expected: Vec<(u32, f64)> = SWEEP_CLOCKS.iter().map(|&c| (c, watts_at(c))).collect();
        assert_eq!(record_points(&all), expected);

        // Clocks commanded in the sweep's descending order.
        assert_eq!(commanded_clocks(&all), SWEEP_CLOCKS.to_vec());

        // Finished carries a 10-point LUT.
        let finished: Vec<&ClockWattsLut> = all
            .iter()
            .filter_map(|e| match e {
                SweepEffect::Finished(lut) => Some(lut),
                _ => None,
            })
            .collect();
        assert_eq!(finished.len(), 1);
        assert_eq!(finished[0].len(), 10);
        assert_eq!(finished[0].watts_for_clock(1200), Some(40.0));
        assert_eq!(finished[0].watts_for_clock(3090), Some(103.0));
    }

    #[test]
    fn sweep_clock_order_is_descending_3090_to_1200() {
        assert_eq!(SWEEP_CLOCKS[0], 3090);
        assert_eq!(SWEEP_CLOCKS[SWEEP_CLOCKS.len() - 1], 1200);
        assert!(SWEEP_CLOCKS.windows(2).all(|w| w[0] > w[1]));
        assert_eq!(SWEEP_CLOCKS.len(), 10);
    }

    #[test]
    fn unpinned_stall_emits_needs_load_once_per_ten() {
        let mut sweep = LutSweep::new();
        sweep.start();
        // 9 idle samples: no transition, no nag yet.
        for _ in 0..9 {
            let effects = sweep.on_sample(&idle());
            assert!(effects.is_empty(), "got {effects:?}");
        }
        assert_eq!(*sweep.state(), SweepState::WaitPinned { clock: 3090 });
        // 10th unpinned sample: exactly one NeedsLoad.
        let effects = sweep.on_sample(&idle());
        assert_eq!(effects, vec![SweepEffect::NeedsLoad]);
        // Next 9: quiet again; 20th: nag again.
        for _ in 0..9 {
            assert!(sweep.on_sample(&idle()).is_empty());
        }
        assert_eq!(sweep.on_sample(&idle()), vec![SweepEffect::NeedsLoad]);
    }

    #[test]
    fn pinned_streak_resets_on_unpinned_sample() {
        let mut sweep = LutSweep::new();
        sweep.start();
        // 2 pinned, 1 idle, 2 pinned: never 3 consecutive -> still waiting.
        sweep.on_sample(&pinned(3090, 100.0));
        sweep.on_sample(&pinned(3090, 100.0));
        sweep.on_sample(&idle());
        sweep.on_sample(&pinned(3090, 100.0));
        sweep.on_sample(&pinned(3090, 100.0));
        assert_eq!(*sweep.state(), SweepState::WaitPinned { clock: 3090 });
        // Third consecutive: Settling.
        sweep.on_sample(&pinned(3090, 100.0));
        assert_eq!(*sweep.state(), SweepState::Settling { clock: 3090 });
    }

    #[test]
    fn invalid_samples_never_count_as_pinned() {
        let mut sweep = LutSweep::new();
        sweep.start();
        // Looks perfectly pinned but a validity flag is down: must not count.
        let mut no_w = pinned(3090, 100.0);
        no_w.gpu_w_valid = false;
        let mut no_mhz = pinned(3090, 100.0);
        no_mhz.gpu_mhz_valid = false;
        for _ in 0..5 {
            sweep.on_sample(&no_w);
            sweep.on_sample(&no_mhz);
        }
        assert_eq!(*sweep.state(), SweepState::WaitPinned { clock: 3090 });
    }

    #[test]
    fn wrong_clock_or_low_util_is_not_pinned() {
        let mut sweep = LutSweep::new();
        sweep.start();
        // Clock off by more than 30 MHz.
        let mut off_clock = pinned(3090, 100.0);
        off_clock.gpu_sm_mhz = 3000.0;
        // Util at 90 exactly (must be > 90).
        let mut low_util = pinned(3090, 100.0);
        low_util.gpu_util_pct = 90.0;
        for _ in 0..5 {
            sweep.on_sample(&off_clock);
            sweep.on_sample(&low_util);
        }
        assert_eq!(*sweep.state(), SweepState::WaitPinned { clock: 3090 });
    }

    #[test]
    fn noisy_then_steady_records_only_after_flat_run() {
        let mut sweep = LutSweep::new();
        sweep.start();
        for _ in 0..PIN_STREAK {
            sweep.on_sample(&pinned(3090, 100.0));
        }
        assert_eq!(*sweep.state(), SweepState::Settling { clock: 3090 });

        // 20 noisy samples alternating +/-3 W (spread 6 > 2 tolerance).
        for i in 0..20 {
            let w = if i % 2 == 0 { 97.0 } else { 103.0 };
            let effects = sweep.on_sample(&pinned(3090, w));
            assert!(effects.is_empty(), "noisy sample {i} recorded: {effects:?}");
        }

        // Flat run: no record until the tail window is entirely flat.
        let mut recorded = Vec::new();
        for i in 0..STEADY_N_GPU_W {
            let effects = sweep.on_sample(&pinned(3090, 100.0));
            if i < STEADY_N_GPU_W - 1 {
                assert!(effects.is_empty(), "sample {i} recorded early: {effects:?}");
            }
            recorded.extend(record_points(&effects));
        }
        assert_eq!(recorded, vec![(3090, 100.0)]);
        assert_eq!(*sweep.state(), SweepState::WaitPinned { clock: 2880 });
        assert_eq!(sweep.progress(), (1, 10));
    }

    #[test]
    fn load_dying_mid_settle_falls_back_and_records_recovery_watts_only() {
        let mut sweep = LutSweep::new();
        sweep.start();
        for _ in 0..PIN_STREAK {
            sweep.on_sample(&pinned(3090, 100.0));
        }
        assert_eq!(*sweep.state(), SweepState::Settling { clock: 3090 });
        // A few settling samples accumulate...
        for _ in 0..5 {
            assert!(sweep.on_sample(&pinned(3090, 100.0)).is_empty());
        }
        // ...then the load dies. Idle watts are VERY flat: without the
        // pinned re-check, 15 of these would record a bogus 3090 MHz -> 15 W
        // point. Instead: straight back to waiting.
        assert!(sweep.on_sample(&idle()).is_empty());
        assert_eq!(*sweep.state(), SweepState::WaitPinned { clock: 3090 });
        for _ in 0..(STEADY_N_GPU_W + 5) {
            let effects = sweep.on_sample(&idle());
            assert!(
                record_points(&effects).is_empty(),
                "idle watts must never record: {effects:?}"
            );
        }
        assert_eq!(sweep.progress(), (0, 10));

        // Recovery: re-pin, then settle at DIFFERENT watts. The recorded
        // point must come from post-recovery samples only.
        for _ in 0..PIN_STREAK {
            assert!(sweep.on_sample(&pinned(3090, 97.0)).is_empty());
        }
        assert_eq!(*sweep.state(), SweepState::Settling { clock: 3090 });
        let mut recorded = Vec::new();
        for _ in 0..STEADY_N_GPU_W {
            recorded.extend(record_points(&sweep.on_sample(&pinned(3090, 97.0))));
        }
        assert_eq!(recorded, vec![(3090, 97.0)]);
        assert_eq!(sweep.progress(), (1, 10));
    }

    #[test]
    fn invalid_sample_mid_settle_also_falls_back() {
        let mut sweep = LutSweep::new();
        sweep.start();
        for _ in 0..PIN_STREAK {
            sweep.on_sample(&pinned(3090, 100.0));
        }
        // Sensor blip: sample looks pinned but the power reading is missing.
        // Trusting the window across an outage risks a poisoned point, so
        // settle restarts from scratch.
        let mut invalid = pinned(3090, 100.0);
        invalid.gpu_w_valid = false;
        assert!(sweep.on_sample(&invalid).is_empty());
        assert_eq!(*sweep.state(), SweepState::WaitPinned { clock: 3090 });
    }

    #[test]
    fn needs_load_nag_rearms_after_mid_settle_fallback() {
        let mut sweep = LutSweep::new();
        sweep.start();
        for _ in 0..PIN_STREAK {
            sweep.on_sample(&pinned(3090, 100.0));
        }
        // Load dies: the fallback sample counts as unpinned #1, so the nag
        // fires on the 9th idle sample after it (10 consecutive unpinned).
        assert!(sweep.on_sample(&idle()).is_empty());
        for _ in 0..8 {
            assert!(sweep.on_sample(&idle()).is_empty());
        }
        assert_eq!(sweep.on_sample(&idle()), vec![SweepEffect::NeedsLoad]);
    }

    #[test]
    fn record_advances_to_next_clock_with_command() {
        let mut sweep = LutSweep::new();
        sweep.start();
        let effects = drive_step(&mut sweep, 3090, 100.0);
        assert_eq!(record_points(&effects), vec![(3090, 100.0)]);
        assert_eq!(commanded_clocks(&effects), vec![2880]);
        assert_eq!(*sweep.state(), SweepState::WaitPinned { clock: 2880 });
    }

    #[test]
    fn watts_window_is_capped() {
        let mut sweep = LutSweep::new();
        sweep.start();
        for _ in 0..PIN_STREAK {
            sweep.on_sample(&pinned(3090, 100.0));
        }
        // Never-steady noise for far longer than the cap: sweep stalls (by
        // design; Esc-abort is the way out) but the window must not grow.
        for i in 0..500 {
            let w = if i % 2 == 0 { 90.0 } else { 110.0 };
            assert!(sweep.on_sample(&pinned(3090, w)).is_empty());
        }
        assert!(sweep.watts_window.len() <= WATTS_WINDOW_CAP);
        assert_eq!(*sweep.state(), SweepState::Settling { clock: 3090 });
    }

    #[test]
    fn done_state_ignores_further_samples() {
        let mut sweep = LutSweep::new();
        sweep.start();
        for &clock in SWEEP_CLOCKS.iter() {
            drive_step(&mut sweep, clock, f64::from(clock) / 30.0);
        }
        assert_eq!(*sweep.state(), SweepState::Done);
        assert!(sweep.on_sample(&pinned(1200, 40.0)).is_empty());
        assert_eq!(sweep.progress(), (10, 10));
    }
}

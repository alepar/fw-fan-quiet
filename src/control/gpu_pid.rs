//! Inner GPU watts→clock loop (design doc §3, research 03-control §3a).
//!
//! Runs at 1 Hz in Auto mode: holds GPU sustained watts at the allocator's
//! setpoint by moving the locked max clock. The calibration LUT provides the
//! feedforward (`clock_for_watts(target)` does the bulk of the work); a light
//! PI — mostly I — trims the residual LUT error and drift on top.
//!
//! # Anti-windup (decision)
//!
//! Clamp-aware **conditional integration**: whenever the *returned* clock was
//! saturated (PI correction clipped to ±`OUTPUT_LIMIT`, clock clipped to the
//! floor/ceiling, or rate-limited), the next update freezes the integrator —
//! it skips `Pid::next_control_output` (which always integrates) and instead
//! recomputes only the P term, reusing the integral contribution the crate
//! reported on the last unsaturated step. Integration resumes on the first
//! unsaturated output. This implements the cascade rule "when the inner loop
//! saturates, stop winding" without back-calculation, which the pid crate
//! does not support.
//!
//! Residual windup is bounded twice over: (a) at most ONE sample of
//! integration can land past saturation (the step that first saturates has
//! already integrated), i.e. ≤ `KI · |error|` MHz; (b) the crate's
//! `i_limit = OUTPUT_LIMIT` caps the integral contribution at ±400 MHz
//! absolutely, so worst-case unwind time is 400 / KI ≈ 27 s of persistent
//! opposite error — acceptable under the allocator's 5 s retarget cadence.
//! The deadband returns early (no integration), so there is no windup from
//! in-deadband noise either.

use crate::control::lut::ClockWattsLut;
use pid::Pid;

/// Proportional gain, MHz per watt of error. Small: the feedforward already
/// lands near the answer; P only speeds up disturbance response.
const KP: f64 = 5.0;
/// Integral gain, MHz per watt per call (1 Hz calls ⇒ per second). Dominant
/// term per the design ("light PI, mostly I"): steady LUT error is trimmed at
/// 15 MHz/s per watt.
const KI: f64 = 15.0;
/// Max PI correction either way, MHz. The feedforward does the bulk; the PI
/// only trims, so its authority is kept tight (also the crate's p/i limits).
const OUTPUT_LIMIT: f64 = 400.0;
/// Hold (return None, no integration) while |measured − target| is within
/// this band, so the loop doesn't chase NVML noise.
const DEADBAND_W: f64 = 3.0;
/// Max clock change per call vs the last returned clock (~1 V/F bin/s).
const RATE_LIMIT_MHZ: u32 = 105;
/// Driver ceiling for the locked max clock (matches actuators::gpu).
const MAX_CLOCK_MHZ: u32 = 3090;

/// GPU watts→clock PI with LUT feedforward. See the module docs.
pub struct GpuPid {
    pid: Pid<f64>,
    /// Last clock this loop returned; rate-limit reference. None after
    /// new()/reset() — the first output may jump straight to the feedforward.
    last_clock: Option<u32>,
    /// True when the last returned clock was saturated (output/floor/ceiling/
    /// rate limited) — the next update freezes the integrator (see module
    /// docs).
    saturated: bool,
    /// Integral contribution reported by the crate on the last unsaturated
    /// step; reused verbatim while frozen.
    frozen_i: f64,
}

// TODO(task-25): constructed and driven by the Auto-mode controller loop;
// dead until then.
#[allow(dead_code)]
impl GpuPid {
    pub fn new() -> Self {
        Self {
            pid: make_pid(0.0),
            last_clock: None,
            saturated: false,
            frozen_i: 0.0,
        }
    }

    /// Update the watts setpoint (allocator, every 5 s). Bumpless: resets
    /// nothing — the integrator keeps its trim and the rate limiter keeps its
    /// reference, so a retarget ramps from the current clock.
    pub fn set_target_w(&mut self, w: f64) {
        self.pid.setpoint = w;
    }

    /// One 1 Hz update: measured GPU watts (validity-gated by the caller) →
    /// desired locked max clock, or None to hold the current clock.
    ///
    /// `clock = lut.clock_for_watts(target) + PI(error)`, then clamped to
    /// `[gpu_floor_mhz, 3090]` and rate-limited to ±105 MHz vs the last
    /// returned clock. Returns None (and does NOT integrate — no windup
    /// inside the deadband) while |measured − target| ≤ 3 W, and also when
    /// the LUT is empty (no calibration → nothing sane to command) or the
    /// measurement is non-finite.
    pub fn update(
        &mut self,
        measured_w: f64,
        lut: &ClockWattsLut,
        gpu_floor_mhz: u32,
    ) -> Option<u32> {
        let error = self.pid.setpoint - measured_w;
        if !error.is_finite() || error.abs() <= DEADBAND_W {
            return None;
        }
        let ff = lut.clock_for_watts(self.pid.setpoint)?;

        let correction = if self.saturated {
            // Conditional integration: previous output was saturated, so
            // freeze the integrator (don't call the crate — it always
            // integrates) and recompute only the P term.
            let p = (error * KP).clamp(-OUTPUT_LIMIT, OUTPUT_LIMIT);
            p + self.frozen_i
        } else {
            let out = self.pid.next_control_output(measured_w);
            self.frozen_i = out.i;
            out.output
        };
        let bounded = correction.clamp(-OUTPUT_LIMIT, OUTPUT_LIMIT);
        let mut saturated = bounded != correction;

        let desired = (f64::from(ff) + bounded).round();
        let clamped = desired.clamp(f64::from(gpu_floor_mhz), f64::from(MAX_CLOCK_MHZ));
        saturated |= clamped != desired;
        // In [gpu_floor_mhz, 3090] by the clamp above: the cast is lossless.
        let mut clock = clamped as u32;

        if let Some(last) = self.last_clock {
            let limited = clock.clamp(
                last.saturating_sub(RATE_LIMIT_MHZ),
                last.saturating_add(RATE_LIMIT_MHZ),
            );
            saturated |= limited != clock;
            // `last` may predate a floor RAISE, in which case the rate limit
            // could hold us below the new floor for a few steps — floors win
            // over rate limits project-wide (allocator does the same), so
            // re-clamp to the floor last.
            clock = limited.max(gpu_floor_mhz);
        }

        self.saturated = saturated;
        self.last_clock = Some(clock);
        Some(clock)
    }

    /// Clear all state (mode exit): integrator, saturation freeze and the
    /// rate-limit reference. Keeps the setpoint (re-entry re-targets anyway).
    pub fn reset(&mut self) {
        self.pid = make_pid(self.pid.setpoint);
        self.last_clock = None;
        self.saturated = false;
        self.frozen_i = 0.0;
    }
}

impl Default for GpuPid {
    fn default() -> Self {
        Self::new()
    }
}

fn make_pid(setpoint: f64) -> Pid<f64> {
    // pid 2.x positional API: (kp, ki, kd, p_limit, i_limit, d_limit,
    // setpoint). No D term. i_limit is the windup backstop (module docs).
    Pid::new(KP, KI, 0.0, OUTPUT_LIMIT, OUTPUT_LIMIT, 0.0, setpoint)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::lut_sweep::SWEEP_CLOCKS;

    const FLOOR: u32 = 1000;

    #[test]
    fn raised_floor_wins_over_rate_limit() {
        // Converge low, then raise the floor far above: the very next output
        // must sit AT the new floor, not ramp up at 105 MHz/step.
        let lut = exact_lut();
        let mut pid = GpuPid::new();
        pid.set_target_w(35.0); // ~1375 MHz on the plant
        let mut clock = 1500;
        for _ in 0..20 {
            if let Some(c) = pid.update(plant(clock), &lut, FLOOR) {
                clock = c;
            }
        }
        assert!(clock < 1500, "premise: converged below 1500");
        let out = pid
            .update(plant(clock), &lut, 2500)
            .expect("floor raise forces a clock change");
        assert_eq!(out, 2500, "floor must win over the rate limit");
    }

    /// Simulated plant: watts drawn at a locked clock. 1500 MHz → 40 W,
    /// 2800 MHz → 92 W. Electrical response is sub-second, so at the 1 Hz
    /// loop cadence the plant is memoryless.
    fn plant(clock: u32) -> f64 {
        0.04 * f64::from(clock) - 20.0
    }

    /// LUT sampled from the SAME plant at the 10 calibration sweep clocks:
    /// feedforward is (interpolation-)exact.
    fn exact_lut() -> ClockWattsLut {
        let mut lut = ClockWattsLut::new();
        for &c in &SWEEP_CLOCKS {
            lut.insert(c, plant(c));
        }
        lut
    }

    /// LUT that reads 10% more watts per clock than the real plant draws:
    /// the feedforward lands low and the integral must trim it out.
    fn miscalibrated_lut() -> ClockWattsLut {
        let mut lut = ClockWattsLut::new();
        for &c in &SWEEP_CLOCKS {
            lut.insert(c, plant(c) * 1.1);
        }
        lut
    }

    /// Deterministic pseudo-noise in [-1, 1) W: inline LCG, no rand crate
    /// (same generator as the thermal_model tests).
    fn lcg_noise(state: &mut u64) -> f64 {
        *state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((*state >> 32) as f64 / f64::from(u32::MAX)) * 2.0 - 1.0
    }

    /// Closed loop around the simulated plant: feeds measured watts
    /// (plant + bias), applies returned clocks.
    struct Sim {
        clock: u32,
    }

    impl Sim {
        fn step(&mut self, pid: &mut GpuPid, lut: &ClockWattsLut, bias_w: f64) -> Option<u32> {
            let out = pid.update(plant(self.clock) + bias_w, lut, FLOOR);
            if let Some(c) = out {
                self.clock = c;
            }
            out
        }

        /// Run until the loop holds (returns None); panics after `max` steps.
        fn settle(&mut self, pid: &mut GpuPid, lut: &ClockWattsLut, max: usize) -> usize {
            for i in 0..max {
                if self.step(pid, lut, 0.0).is_none() {
                    return i;
                }
            }
            panic!(
                "did not settle within {max} iterations (clock {})",
                self.clock
            );
        }
    }

    #[test]
    fn converges_with_exact_lut_within_10() {
        let lut = exact_lut();
        let mut pid = GpuPid::new();
        pid.set_target_w(60.0);
        let mut sim = Sim { clock: 1500 };
        sim.settle(&mut pid, &lut, 10);
        let w = plant(sim.clock);
        assert!(
            (w - 60.0).abs() <= 3.0,
            "settled at {w} W (clock {})",
            sim.clock
        );
    }

    #[test]
    fn miscalibrated_lut_integral_trims_within_40() {
        let lut = miscalibrated_lut();
        // Test premise: feedforward alone must be off by more than the
        // deadband, or the PI has nothing to prove.
        let ff = lut.clock_for_watts(60.0).unwrap();
        assert!(
            (plant(ff) - 60.0).abs() > 3.0,
            "premise broken: FF alone already lands at {} W",
            plant(ff)
        );
        let mut pid = GpuPid::new();
        pid.set_target_w(60.0);
        let mut sim = Sim { clock: 1500 };
        sim.settle(&mut pid, &lut, 40);
        let w = plant(sim.clock);
        assert!(
            (w - 60.0).abs() <= 3.0,
            "settled at {w} W (clock {})",
            sim.clock
        );
    }

    #[test]
    fn deadband_holds_and_no_windup_on_disturbance() {
        let lut = exact_lut();
        let mut pid = GpuPid::new();
        pid.set_target_w(60.0);
        let mut sim = Sim { clock: 1500 };
        sim.settle(&mut pid, &lut, 10);
        let held = sim.clock;

        // 20 in-deadband samples: no clock updates, and (crucially) no
        // integration — verified below via the disturbance response.
        for _ in 0..20 {
            assert_eq!(sim.step(&mut pid, &lut, 0.0), None);
            assert_eq!(sim.clock, held);
        }

        // +6 W disturbance. If the integrator had drifted during the 20
        // deadband samples (20 × 3 W × KI would peg it at ±400), the loop
        // would slam ~400 MHz down; the true correction is ≈ 6/0.04 = 150.
        let mut min_clock = held;
        let mut first = None;
        let mut settled = false;
        for _ in 0..15 {
            match sim.step(&mut pid, &lut, 6.0) {
                Some(c) => {
                    first.get_or_insert(c);
                    min_clock = min_clock.min(c);
                }
                None => {
                    settled = true;
                    break;
                }
            }
        }
        assert!(settled, "disturbance response did not re-settle");
        let first = first.expect("disturbance must produce a correction");
        assert!(first < held, "correction must move the clock down");
        // Proportional-sized first step: ≤ (KP+KI)·6 = 120 MHz of fresh PI
        // (the rate limit may cap it lower), not a windup jump.
        assert!(held - first <= 120, "first correction {} MHz", held - first);
        assert!(
            min_clock >= held - 250,
            "windup jump: sank to {min_clock} from {held}"
        );
    }

    #[test]
    fn rate_limited_on_target_jump() {
        let lut = exact_lut();
        let mut pid = GpuPid::new();
        pid.set_target_w(30.0);
        // Start off-target so convergence actually emits clocks: the rate
        // limiter is referenced to the last RETURNED clock, and a loop that
        // never commanded anything is allowed its documented first-output
        // jump straight to the feedforward.
        let mut sim = Sim { clock: 1500 };
        sim.settle(&mut pid, &lut, 10);

        pid.set_target_w(90.0);
        let mut prev = sim.clock;
        let mut settled = false;
        for _ in 0..40 {
            match sim.step(&mut pid, &lut, 0.0) {
                Some(c) => {
                    assert!(
                        c.abs_diff(prev) <= RATE_LIMIT_MHZ,
                        "step {prev} → {c} exceeds the rate limit"
                    );
                    prev = c;
                }
                None => {
                    settled = true;
                    break;
                }
            }
        }
        assert!(settled, "did not settle after the target jump");
        let w = plant(sim.clock);
        assert!((w - 90.0).abs() <= 3.0, "settled at {w} W");
    }

    #[test]
    fn floor_respected_no_oscillation() {
        let lut = exact_lut();
        let mut pid = GpuPid::new();
        // 5 W is below what the floor clock draws (plant(1000) = 20 W).
        pid.set_target_w(5.0);
        let mut sim = Sim { clock: 1500 };
        let mut outputs = Vec::new();
        for _ in 0..30 {
            let out = sim.step(&mut pid, &lut, 0.0);
            if let Some(c) = out {
                assert!(c >= FLOOR, "clock {c} below the floor");
            }
            outputs.push(out);
        }
        assert_eq!(
            sim.clock, FLOOR,
            "unreachable-low target must pin the floor"
        );
        // Stable: last 5 outputs identical (all None or all the same clock).
        let tail = &outputs[outputs.len() - 5..];
        assert!(
            tail.iter().all(|o| o == &tail[0]),
            "oscillating at the floor: {tail:?}"
        );
    }

    #[test]
    fn windup_bounded_recovery_from_floor() {
        let lut = exact_lut();
        let mut pid = GpuPid::new();
        pid.set_target_w(5.0);
        let mut sim = Sim { clock: 1500 };
        for _ in 0..100 {
            sim.step(&mut pid, &lut, 0.0);
        }
        assert_eq!(sim.clock, FLOOR);

        // Documented windup bound: recovery within 400/KI ≈ 27 calls;
        // conditional integration makes it much faster in practice.
        pid.set_target_w(60.0);
        sim.settle(&mut pid, &lut, 30);
        let w = plant(sim.clock);
        assert!(
            (w - 60.0).abs() <= 3.0,
            "recovered to {w} W (clock {})",
            sim.clock
        );
    }

    #[test]
    fn retarget_is_bumpless() {
        let lut = exact_lut();
        let mut pid = GpuPid::new();
        pid.set_target_w(60.0);
        let mut sim = Sim { clock: 1500 };
        sim.settle(&mut pid, &lut, 10);
        let converged = sim.clock;

        // Retarget upward: the approach must be monotone-ish — never dip
        // below the converged clock by more than one rate-limit step.
        pid.set_target_w(70.0);
        let mut min_clock = converged;
        let mut settled = false;
        for _ in 0..20 {
            match sim.step(&mut pid, &lut, 0.0) {
                Some(c) => min_clock = min_clock.min(c),
                None => {
                    settled = true;
                    break;
                }
            }
        }
        assert!(settled, "did not settle after retarget");
        assert!(
            min_clock >= converged - RATE_LIMIT_MHZ,
            "transient dipped to {min_clock} (converged {converged})"
        );
        let w = plant(sim.clock);
        assert!((w - 70.0).abs() <= 3.0, "settled at {w} W");
    }

    #[test]
    fn reset_clears_integrator_and_rate_reference() {
        let lut = exact_lut();

        // Integrator cleared: after converging (integrator holds some trim),
        // reset and probe with a known error — the output must be exactly
        // feedforward + one fresh P+I sample.
        let mut pid = GpuPid::new();
        pid.set_target_w(60.0);
        let mut sim = Sim { clock: 1500 };
        sim.settle(&mut pid, &lut, 10);
        pid.reset();
        // FF(60) = 2000 (exact LUT); error 5 → P 25 + I 75 → 2100.
        assert_eq!(pid.update(55.0, &lut, FLOOR), Some(2100));

        // Rate reference cleared: the first post-reset output may jump far
        // more than 105 MHz from the pre-reset clock.
        let mut pid = GpuPid::new();
        pid.set_target_w(60.0);
        // error 20 → P 100 + I 300 → FF 2000 + 400 = 2400.
        assert_eq!(pid.update(40.0, &lut, FLOOR), Some(2400));
        pid.reset();
        // error −30 → P −150, I clamps to −400, output clamps to −400 →
        // 1600: an 800 MHz jump (no rate limit), and fresh integral (stale
        // +300 would land at 1700).
        assert_eq!(pid.update(90.0, &lut, FLOOR), Some(1600));
    }

    #[test]
    fn noisy_measurements_no_churn() {
        let lut = exact_lut();
        let mut pid = GpuPid::new();
        pid.set_target_w(60.0);
        let mut sim = Sim { clock: 1500 };
        let mut seed = 0xbeef_cafe_u64;
        let mut clocks = Vec::new();
        for _ in 0..60 {
            sim.step(&mut pid, &lut, lcg_noise(&mut seed));
            clocks.push(sim.clock);
        }
        // Converged despite ±1 W noise (deadband may hold up to 3 W of true
        // error plus the noise the sample carried).
        let w = plant(sim.clock);
        assert!((w - 60.0).abs() <= 4.0, "settled at {w} W");
        // No churn: the last 20 applied clocks stay within one rate step.
        let tail = &clocks[40..];
        let (min, max) = (tail.iter().min().unwrap(), tail.iter().max().unwrap());
        assert!(max - min <= RATE_LIMIT_MHZ, "churning: {min}..{max}");
    }

    #[test]
    fn empty_lut_returns_none() {
        let lut = ClockWattsLut::new();
        let mut pid = GpuPid::new();
        pid.set_target_w(60.0);
        assert_eq!(pid.update(20.0, &lut, FLOOR), None);
    }
}

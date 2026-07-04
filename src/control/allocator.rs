//! Demand estimator + contour allocator (design doc §3, research 03 §3–4):
//! the policy heart of the closed loop. Every 5 s the controller (Task 25)
//! inverts the thermal model to the ≤target-RPM contour in the (cpu_w, gpu_w)
//! plane and this allocator picks the operating point on it, weighted by
//! per-device starvation scores.
//!
//! Safety invariants (these outrank optimality):
//! - The CPU floor wins over everything, including rate limits.
//! - A lost fan sensor must never raise power: fan invalid → freeze at the
//!   last commanded point (or the conservative start).
//! - Asymmetric rate limits: power rises slowly (creeping up on the noise
//!   ceiling is never urgent) but falls fast — and *twice* as fast on RPM
//!   overshoot, because the acoustic contract is already broken and every
//!   extra second over target is audible. Ups are forbidden entirely while
//!   overshooting.
//!
//! Floors simplification: the allocator enforces only the CPU floor. The GPU
//! floor is a *clock* floor (MHz) and lives in the watts→clock PI's clamp
//! (Task 24) — the allocator's gpu_w output is a setpoint for that loop, so
//! clamping watts here would just fight the clock clamp there.

// TODO(task-25): consumed by the auto-mode controller loop; dead until then.
#![allow(dead_code)]

use crate::types::Sample;

/// Grid resolution of the contour search over cpu_w.
pub const GRID_STEP_W: f64 = 0.5;
/// Max per-device power increase per allocator step (slow creep up).
pub const UP_RATE_W: f64 = 2.0;
/// Max per-device power decrease per allocator step (fast back-off).
pub const DOWN_RATE_W: f64 = 8.0;
/// Max per-device decrease while the fan overshoots the target. Doubled vs
/// DOWN_RATE_W: overshoot means the acoustic contract is already violated,
/// so cutting hard beats staying audibly loud (research 03 §3 asymmetry).
pub const OVERSHOOT_DOWN_RATE_W: f64 = 16.0;
/// RPM deadband around the fan target (≈ just-noticeable difference).
pub const DEADBAND_RPM: f64 = 150.0;
/// Power deadband: candidate moves smaller than this (on both axes, while
/// RPM is in band) are noise, not demand shifts — hold the last point.
pub const DEADBAND_W: f64 = 1.5;
/// First commanded point before any history exists: low enough to be quiet
/// on any sane calibration, high enough not to stall the desktop.
pub const CONSERVATIVE_START: (f64, f64) = (15.0, 30.0);
/// CPU sustained ceiling (cTDP max, design §1).
pub const CPU_MAX_W: f64 = 54.0;
/// GPU ceiling (module AC TGP, design §1).
pub const GPU_MAX_W: f64 = 100.0;
/// CPU pinned-at-limit margin: measured within this of the cap → starved.
pub const CPU_PINNED_MARGIN_W: f64 = 1.5;
/// GPU pinned-at-clock margin: sm clock within this of the commanded max
/// (combined with high utilization) → starved by our cap.
pub const GPU_PINNED_MARGIN_MHZ: f64 = 30.0;
/// Utilization gate for the GPU pinned boost: clock-pinned but idle is
/// "nothing to do", not starvation.
pub const GPU_PINNED_UTIL_PCT: f64 = 90.0;

/// Per-device starvation scores, each 0..=1 (research 03 §4: draw/allowed
/// ratio + pinned-at-limit boolean + utilization fallback).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Demand {
    pub cpu_starved: f64,
    pub gpu_starved: f64,
}

/// Starvation score per device: how close is measured draw to the allowed cap.
///
/// CPU: `min(1, cpu_pkg_w / cpu_limit_w)` — but only when a limit is active
/// and the sample is valid (`cpu_pkg_w > 0`); no limit (or a dead RAPL
/// reading) → utilization fallback (`cpu_util_pct/100`, capped at 1).
/// Pinned boost: measured within [`CPU_PINNED_MARGIN_W`] of the cap → 1.0
/// exactly (it wants more).
///
/// GPU: `min(1, gpu_w / gpu_target_w)` when a target > 0 is set and the NVML
/// power reading is valid, else `gpu_util_pct/100`. Pinned boost: sm clock
/// within [`GPU_PINNED_MARGIN_MHZ`] of the commanded max AND utilization
/// above [`GPU_PINNED_UTIL_PCT`] → 1.0.
pub fn demand(
    s: &Sample,
    cpu_limit_w: Option<f64>,
    gpu_target_w: Option<f64>,
    gpu_max_mhz: Option<u32>,
) -> Demand {
    let cpu_starved = match cpu_limit_w {
        Some(limit) if limit > 0.0 && s.cpu_pkg_w > 0.0 => {
            if limit - s.cpu_pkg_w <= CPU_PINNED_MARGIN_W {
                1.0
            } else {
                (s.cpu_pkg_w / limit).min(1.0)
            }
        }
        _ => (s.cpu_util_pct / 100.0).clamp(0.0, 1.0),
    };

    let gpu_pinned = gpu_max_mhz.is_some_and(|max| {
        s.gpu_mhz_valid
            && f64::from(max) - s.gpu_sm_mhz <= GPU_PINNED_MARGIN_MHZ
            && s.gpu_util_pct > GPU_PINNED_UTIL_PCT
    });
    let gpu_starved = if gpu_pinned {
        1.0
    } else {
        match gpu_target_w {
            Some(t) if t > 0.0 && s.gpu_w_valid => (s.gpu_w / t).clamp(0.0, 1.0),
            _ => (s.gpu_util_pct / 100.0).clamp(0.0, 1.0),
        }
    };

    Demand {
        cpu_starved,
        gpu_starved,
    }
}

/// One allocator step's inputs.
pub struct AllocInput<'a> {
    /// pc → pg on the target-RPM contour (`ThermalModel::gpu_watts_on_contour`
    /// with trim applied by the caller). `None` = degenerate at this pc.
    pub contour: &'a dyn Fn(f64) -> Option<f64>,
    pub demand: Demand,
    /// (cpu_floor_w, gpu_floor_mhz). Only the CPU floor is enforced here; the
    /// GPU clock floor belongs to the watts→clock PI clamp (Task 24) — see
    /// the module docs.
    pub floors: (f64, u32),
    /// Current `max(fan1, fan2)` RPM, for the deadband/overshoot checks.
    pub measured_fan_rpm: f64,
    pub fan_target_rpm: f64,
    pub fan_valid: bool,
}

/// Picks (cpu_w, gpu_w) on the contour honoring the demand split, deadband
/// and asymmetric rate limits. Pure policy: no hardware, no clock.
#[derive(Debug, Clone, Default)]
pub struct Allocator {
    /// Last commanded (cpu_w, gpu_w); None before the first step / after reset.
    last: Option<(f64, f64)>,
}

impl Allocator {
    pub fn new() -> Self {
        Self::default()
    }

    /// One allocation step (called every 5 s in Auto mode). Policy:
    /// 1. Fan invalid → freeze: return the last commanded point (conservative
    ///    start on the first call). A lost fan sensor must never raise power.
    /// 2. Grid-search pc over [cpu_floor, CPU_MAX_W] in GRID_STEP_W steps;
    ///    candidates (pc, contour(pc)) with pg clamped to [0, GPU_MAX_W],
    ///    pc skipped where the contour is degenerate (None). No candidates →
    ///    freeze as in 1.
    /// 3. Score = cpu_starved·pc/CPU_MAX_W + gpu_starved·pg/GPU_MAX_W, ties
    ///    broken toward GPU (gaming default).
    /// 4. Deadband: RPM within ±DEADBAND_RPM of target AND the candidate
    ///    within DEADBAND_W of the last point on both axes → hold.
    /// 5. Rate limits vs the last commanded point: up ≤ UP_RATE_W, down ≤
    ///    DOWN_RATE_W. RPM overshoot (measured > target + DEADBAND_RPM) →
    ///    ups forbidden, down ≤ OVERSHOOT_DOWN_RATE_W.
    /// 6. cpu_w is clamped to ≥ cpu_floor last — floors win over everything.
    pub fn step(&mut self, inp: &AllocInput) -> (f64, f64) {
        let prev = self.last.unwrap_or(CONSERVATIVE_START);
        self.last = Some(prev);
        if !inp.fan_valid {
            return prev;
        }

        let cpu_floor = inp.floors.0;
        let Some((cand_pc, cand_pg)) = best_candidate(inp.contour, inp.demand, cpu_floor) else {
            return prev; // degenerate contour everywhere → freeze
        };

        let rpm_err = inp.measured_fan_rpm - inp.fan_target_rpm;
        if rpm_err.abs() <= DEADBAND_RPM
            && (cand_pc - prev.0).abs() < DEADBAND_W
            && (cand_pg - prev.1).abs() < DEADBAND_W
        {
            return prev;
        }

        let (up, down) = if rpm_err > DEADBAND_RPM {
            (0.0, OVERSHOOT_DOWN_RATE_W) // overshoot: cut hard, never raise
        } else {
            (UP_RATE_W, DOWN_RATE_W)
        };
        let out = (
            cand_pc.clamp(prev.0 - down, prev.0 + up).max(cpu_floor),
            cand_pg.clamp(prev.1 - down, prev.1 + up),
        );
        self.last = Some(out);
        out
    }

    /// Forget history (on mode exit): the next step starts from the
    /// conservative point again.
    pub fn reset(&mut self) {
        self.last = None;
    }
}

/// Max-utility candidate on the grid. Assumes the physical contour is
/// non-increasing in pc, so scanning pc upward and keeping strictly-better
/// scores breaks exact ties toward the lowest pc = highest pg (GPU).
fn best_candidate(
    contour: &dyn Fn(f64) -> Option<f64>,
    d: Demand,
    cpu_floor: f64,
) -> Option<(f64, f64)> {
    let mut best: Option<((f64, f64), f64)> = None;
    let mut pc = cpu_floor;
    while pc <= CPU_MAX_W {
        if let Some(pg_raw) = contour(pc) {
            let pg = pg_raw.clamp(0.0, GPU_MAX_W);
            let score = d.cpu_starved * (pc / CPU_MAX_W) + d.gpu_starved * (pg / GPU_MAX_W);
            if best.is_none_or(|(_, s)| score > s) {
                best = Some(((pc, pg), score));
            }
        }
        pc += GRID_STEP_W; // 0.5 is binary-exact: no drift over the grid
    }
    best.map(|(cand, _)| cand)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Linear ground-truth contour (thermal_model tests' truth params):
    /// pg = (target − 800 − 25·pc) / (15 + 0.1·pc), clamped ≥ 0 like
    /// `gpu_watts_on_contour` does.
    fn contour_for(target: f64) -> impl Fn(f64) -> Option<f64> {
        move |pc| Some(((target - 800.0 - 25.0 * pc) / (15.0 + 0.1 * pc)).max(0.0))
    }

    /// Same grid + tie-break as the allocator, computed independently.
    fn brute_force_best(contour: &dyn Fn(f64) -> Option<f64>, d: Demand) -> (f64, f64) {
        let mut best = (f64::NAN, f64::NAN);
        let mut best_score = f64::NEG_INFINITY;
        for i in 0.. {
            let pc = 15.0 + f64::from(i) * 0.5;
            if pc > 54.0 {
                break;
            }
            let Some(pg) = contour(pc) else { continue };
            let pg = pg.clamp(0.0, 100.0);
            let score = d.cpu_starved * pc / 54.0 + d.gpu_starved * pg / 100.0;
            if score > best_score {
                best_score = score;
                best = (pc, pg);
            }
        }
        best
    }

    fn inp<'a>(
        contour: &'a dyn Fn(f64) -> Option<f64>,
        demand: Demand,
        measured: f64,
        target: f64,
    ) -> AllocInput<'a> {
        AllocInput {
            contour,
            demand,
            floors: (15.0, 1000),
            measured_fan_rpm: measured,
            fan_target_rpm: target,
            fan_valid: true,
        }
    }

    const D_BOTH: Demand = Demand {
        cpu_starved: 1.0,
        gpu_starved: 1.0,
    };
    const D_GPU: Demand = Demand {
        cpu_starved: 0.2,
        gpu_starved: 1.0,
    };
    const D_GPU_ONLY: Demand = Demand {
        cpu_starved: 0.0,
        gpu_starved: 1.0,
    };

    // ---- allocator: contour policy -------------------------------------

    #[test]
    fn gpu_starved_shifts_toward_gpu() {
        let c = contour_for(3000.0);
        let mut a = Allocator::new();
        // Fan well below target: out of deadband, not overshooting.
        let i = inp(&c, D_GPU, 1700.0, 2000.0);
        let mut prev = CONSERVATIVE_START;
        for _ in 0..10 {
            let out = a.step(&i);
            assert!(
                out.1 > prev.1,
                "gpu_w must rise monotonically: {} -> {}",
                prev.1,
                out.1
            );
            assert!(out.1 - prev.1 <= UP_RATE_W + 1e-9);
            prev = out;
        }
        // 10 steps × 2 W from 30 → GPU at 50 while CPU stays near its floor.
        assert!((prev.1 - 50.0).abs() < 1e-9, "gpu_w = {}", prev.1);
        let expected = brute_force_best(&c, D_GPU);
        assert!(prev.0 <= expected.0 + 1e-9);
        // And with enough steps it converges exactly onto the optimum.
        for _ in 0..40 {
            prev = a.step(&i);
        }
        assert_eq!(prev, expected);
        assert!(expected.1 >= 99.0, "optimum should sit at the GPU end");
    }

    #[test]
    fn both_starved_converges_to_grid_utility_max() {
        let c = contour_for(3000.0);
        let expected = brute_force_best(&c, D_BOTH);
        let mut a = Allocator::new();
        let i = inp(&c, D_BOTH, 1700.0, 2000.0);
        let mut out = CONSERVATIVE_START;
        for _ in 0..60 {
            out = a.step(&i);
        }
        assert_eq!(out, expected, "must converge onto the brute-force optimum");
    }

    #[test]
    fn up_rate_limited_from_conservative_start() {
        let c = contour_for(3000.0); // far-away optimum
        let mut a = Allocator::new();
        let i = inp(&c, D_BOTH, 1700.0, 2000.0);
        let mut prev = CONSERVATIVE_START;
        for step in 0..20 {
            let out = a.step(&i);
            assert!(
                out.0 - prev.0 <= UP_RATE_W + 1e-9,
                "step {step}: cpu up {} -> {}",
                prev.0,
                out.0
            );
            assert!(
                out.1 - prev.1 <= UP_RATE_W + 1e-9,
                "step {step}: gpu up {} -> {}",
                prev.1,
                out.1
            );
            prev = out;
        }
    }

    #[test]
    fn down_rate_limited_on_contour_shrink() {
        let rich = contour_for(3000.0);
        let mut a = Allocator::new();
        let mut out = (0.0, 0.0);
        for _ in 0..60 {
            out = a.step(&inp(&rich, D_GPU, 1700.0, 2000.0));
        }
        assert!(out.1 >= 90.0, "precondition: converged high, gpu={}", out.1);
        // Target collapses (user lowered fan target): contour shrinks hard.
        let poor = contour_for(1200.0);
        let mut prev = out;
        for step in 0..20 {
            let now = a.step(&inp(&poor, D_GPU, 1700.0, 2000.0));
            assert!(
                prev.1 - now.1 <= DOWN_RATE_W + 1e-9,
                "step {step}: gpu drop {} -> {}",
                prev.1,
                now.1
            );
            assert!(now.1 <= prev.1 + 1e-9, "gpu must not rise on a shrink");
            prev = now;
        }
        // First shrink step drops by exactly the full down rate.
        let mut b = Allocator::new();
        for _ in 0..60 {
            b.step(&inp(&rich, D_GPU, 1700.0, 2000.0));
        }
        let first = b.step(&inp(&poor, D_GPU, 1700.0, 2000.0));
        assert!((out.1 - first.1 - DOWN_RATE_W).abs() < 1e-9);
    }

    // ---- allocator: overshoot asymmetry ---------------------------------

    #[test]
    fn overshoot_cuts_at_double_rate_and_never_raises() {
        let rich = contour_for(3000.0);
        let mut a = Allocator::new();
        let mut settled = (0.0, 0.0);
        for _ in 0..60 {
            settled = a.step(&inp(&rich, D_BOTH, 1700.0, 2000.0));
        }
        // Fan overshoots by 300 RPM while the contour shrinks: down steps of
        // up to 16 W are allowed...
        let poor = contour_for(1200.0);
        let over = a.step(&inp(&poor, D_BOTH, 2300.0, 2000.0));
        assert!((settled.1 - over.1 - OVERSHOOT_DOWN_RATE_W).abs() < 1e-9);
        // ...but ups NEVER happen, even though the candidate (54, 0) and the
        // richer contour both want more CPU.
        assert!(over.0 <= settled.0 + 1e-9, "cpu rose during overshoot");
        let richer = contour_for(4000.0);
        let prev = over;
        let out = a.step(&inp(&richer, D_BOTH, 2300.0, 2000.0));
        assert!(out.0 <= prev.0 + 1e-9, "cpu rose during overshoot");
        assert!(out.1 <= prev.1 + 1e-9, "gpu rose during overshoot");
    }

    // ---- allocator: deadband ---------------------------------------------

    #[test]
    fn deadband_holds_last_point() {
        // GPU-only demand on a flat contour ties every pc → tie-break toward
        // GPU picks the floor: candidate (15, 30.5), within DEADBAND_W of
        // the conservative start on both axes.
        let flat = |_pc: f64| Some(30.5);
        let mut a = Allocator::new();
        // Fan within 150 RPM of target → hold, byte-identical output.
        let first = a.step(&inp(&flat, D_GPU_ONLY, 2100.0, 2000.0));
        assert_eq!(first, CONSERVATIVE_START);
        let second = a.step(&inp(&flat, D_GPU_ONLY, 1900.0, 2000.0));
        assert_eq!(second, first, "deadband must hold the last point");
        // Out of the RPM band the same candidate is applied (gpu 30 → 30.5).
        let third = a.step(&inp(&flat, D_GPU_ONLY, 1700.0, 2000.0));
        assert_eq!(third, (15.0, 30.5));
    }

    #[test]
    fn deadband_needs_both_axes_close() {
        // GPU candidate is 5 W away: RPM in band, but this is a real demand
        // shift, not noise → must move.
        let flat = |_pc: f64| Some(35.0);
        let mut a = Allocator::new();
        let out = a.step(&inp(&flat, D_GPU_ONLY, 2000.0, 2000.0));
        assert_eq!(out, (15.0, 32.0)); // rate-limited toward (15, 35)
    }

    // ---- allocator: floors ----------------------------------------------

    #[test]
    fn cpu_never_below_floor() {
        // Steep contour rewarding tiny pc; GPU-only demand pulls pc down.
        let steep = |pc: f64| Some((120.0 - 2.0 * pc).max(0.0));
        let mut a = Allocator::new();
        for _ in 0..30 {
            let (cpu, _) = a.step(&inp(&steep, D_GPU, 1700.0, 2000.0));
            assert!(cpu >= 15.0, "cpu_w {cpu} fell below the floor");
        }
    }

    #[test]
    fn raised_floor_wins_over_up_rate_limit() {
        let c = contour_for(3000.0);
        let mut a = Allocator::new();
        let mut i = inp(&c, D_BOTH, 1700.0, 2000.0);
        i.floors = (20.0, 1000);
        // From the conservative start (15 W) the up-rate limit alone would
        // allow only 17 W; the floor overrides it immediately.
        let (cpu, _) = a.step(&i);
        assert!((cpu - 20.0).abs() < 1e-9, "cpu_w = {cpu}");
    }

    // ---- allocator: fan invalid → freeze ---------------------------------

    #[test]
    fn fan_invalid_freezes() {
        let c = contour_for(3000.0);
        let mut a = Allocator::new();
        let mut i = inp(&c, D_BOTH, 1700.0, 2000.0);
        i.fan_valid = false;
        // First call: conservative start, regardless of a starving demand.
        assert_eq!(a.step(&i), CONSERVATIVE_START);
        assert_eq!(a.step(&i), CONSERVATIVE_START);
        // Move somewhere, then lose the sensor: frozen at the last point.
        i.fan_valid = true;
        let mut last = (0.0, 0.0);
        for _ in 0..5 {
            last = a.step(&i);
        }
        assert_ne!(last, CONSERVATIVE_START);
        i.fan_valid = false;
        assert_eq!(a.step(&i), last);
        assert_eq!(a.step(&i), last);
    }

    #[test]
    fn degenerate_contour_freezes() {
        let none = |_pc: f64| None;
        let mut a = Allocator::new();
        let out = a.step(&inp(&none, D_BOTH, 1700.0, 2000.0));
        assert_eq!(out, CONSERVATIVE_START);
    }

    // ---- allocator: reset -------------------------------------------------

    #[test]
    fn reset_restarts_from_conservative_start() {
        let c = contour_for(3000.0);
        let mut a = Allocator::new();
        for _ in 0..10 {
            a.step(&inp(&c, D_BOTH, 1700.0, 2000.0));
        }
        a.reset();
        let out = a.step(&inp(&c, D_BOTH, 1700.0, 2000.0));
        assert!(out.0 <= CONSERVATIVE_START.0 + UP_RATE_W + 1e-9);
        assert!(out.1 <= CONSERVATIVE_START.1 + UP_RATE_W + 1e-9);
    }

    // ---- demand() ----------------------------------------------------------

    fn sample() -> Sample {
        Sample {
            cpu_pkg_w: 20.0,
            cpu_util_pct: 40.0,
            gpu_w: 40.0,
            gpu_w_valid: true,
            gpu_sm_mhz: 1500.0,
            gpu_mhz_valid: true,
            gpu_util_pct: 60.0,
            ..Sample::default()
        }
    }

    #[test]
    fn demand_cap_ratio() {
        let d = demand(&sample(), Some(40.0), Some(80.0), None);
        assert!((d.cpu_starved - 0.5).abs() < 1e-9);
        assert!((d.gpu_starved - 0.5).abs() < 1e-9);
    }

    #[test]
    fn demand_ratio_capped_at_one() {
        let mut s = sample();
        s.cpu_pkg_w = 50.0; // bursting over a 40 W limit
        s.gpu_w = 90.0;
        let d = demand(&s, Some(40.0), Some(80.0), None);
        assert_eq!(d.cpu_starved, 1.0);
        assert_eq!(d.gpu_starved, 1.0);
    }

    #[test]
    fn demand_cpu_pinned_boost() {
        let mut s = sample();
        s.cpu_pkg_w = 38.6; // within 1.5 W of the 40 W cap
        let d = demand(&s, Some(40.0), Some(80.0), None);
        assert_eq!(d.cpu_starved, 1.0);
        s.cpu_pkg_w = 38.4; // just outside the margin → plain ratio
        let d = demand(&s, Some(40.0), Some(80.0), None);
        assert!((d.cpu_starved - 0.96).abs() < 1e-9);
    }

    #[test]
    fn demand_gpu_pinned_boost() {
        let mut s = sample();
        s.gpu_w = 20.0; // draw ratio alone would say 0.25...
        s.gpu_sm_mhz = 1980.0; // ...but the clock sits at the 2000 MHz lock
        s.gpu_util_pct = 95.0;
        let d = demand(&s, Some(40.0), Some(80.0), Some(2000));
        assert_eq!(d.gpu_starved, 1.0);
        // Clock-pinned but idle (util ≤ 90) is not starvation.
        s.gpu_util_pct = 85.0;
        let d = demand(&s, Some(40.0), Some(80.0), Some(2000));
        assert!((d.gpu_starved - 0.25).abs() < 1e-9);
        // Clock well below the lock → demand-limited, no boost.
        s.gpu_util_pct = 95.0;
        s.gpu_sm_mhz = 1900.0;
        let d = demand(&s, Some(40.0), Some(80.0), Some(2000));
        assert!((d.gpu_starved - 0.25).abs() < 1e-9);
        // Invalid clock reading → no boost off a phantom pin.
        s.gpu_sm_mhz = 1980.0;
        s.gpu_mhz_valid = false;
        let d = demand(&s, Some(40.0), Some(80.0), Some(2000));
        assert!((d.gpu_starved - 0.25).abs() < 1e-9);
    }

    #[test]
    fn demand_fallbacks_without_limits() {
        // No caps active → utilization is the only signal.
        let d = demand(&sample(), None, None, None);
        assert!((d.cpu_starved - 0.4).abs() < 1e-9);
        assert!((d.gpu_starved - 0.6).abs() < 1e-9);
        // Utilization above 100 (rounding glitches) still capped at 1.
        let mut s = sample();
        s.cpu_util_pct = 130.0;
        s.gpu_util_pct = 130.0;
        let d = demand(&s, None, None, None);
        assert_eq!(d.cpu_starved, 1.0);
        assert_eq!(d.gpu_starved, 1.0);
    }

    #[test]
    fn demand_invalid_samples_fall_back_to_utilization() {
        // Dead RAPL (pkg 0) with a limit set → utilization fallback.
        let mut s = sample();
        s.cpu_pkg_w = 0.0;
        let d = demand(&s, Some(40.0), Some(80.0), None);
        assert!((d.cpu_starved - 0.4).abs() < 1e-9);
        // Invalid NVML power with a target set → utilization fallback.
        let mut s = sample();
        s.gpu_w_valid = false;
        let d = demand(&s, Some(40.0), Some(80.0), None);
        assert!((d.gpu_starved - 0.6).abs() < 1e-9);
    }
}

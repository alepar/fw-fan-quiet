//! Demand estimator + scalar budget split (design doc §3.1): the allocator's
//! job shrinks to one arithmetic step now that a single integrator
//! (`control/budget.rs`, `fw-fanctrl-loop-j6s`) owns the closed-loop policy
//! that used to live here (a per-tick search for the target-RPM operating
//! point, plus the deadband/velocity/drain-recovery/taper machinery that
//! search needed). This module now only:
//! - estimates per-device starvation ([`demand`], unchanged), and
//! - splits a scalar power budget between the two axes ([`split_budget`]),
//!   honoring both floors first and handing surplus one axis cannot absorb
//!   (it is at its cap) to the other.
//!
//! Safety invariants that still apply here:
//! - Both floors always win: `split_budget` returns at least the floor on
//!   each axis regardless of how small `budget_w` is, and [`Allocator::step`]
//!   re-applies the (possibly just-raised) floor after both the slew clamp
//!   and the grid quantisation — last, and unquantised — so a floor raised
//!   between steps lifts its axis immediately rather than waiting out
//!   [`UP_RATE_W`], and a floor that isn't itself grid-aligned is still met
//!   exactly rather than getting quantised back down below itself.
//! - Asymmetric rate limits: power rises slowly ([`UP_RATE_W`]) but falls
//!   fast ([`DOWN_RATE_W`]) — creeping up on the noise ceiling is never
//!   urgent, backing off from an over-budget point is. This is now a plain
//!   safety bound on the split's output; the integrator's own gains set the
//!   real pace (design §2.4).
//!
//! Everything else that used to live in this module — the search itself and
//! every band/gate/recovery mechanism it needed, plus the fixed first-command
//! seed point — is deleted with this task (`fw-fanctrl-loop-zct`): the single
//! integrator's clamping anti-windup (design §2.4) now does the job those
//! mechanisms did, one level up.

use crate::types::Sample;

/// Grid resolution the split's output is quantised to.
pub const GRID_STEP_W: f64 = 0.5;
/// Max per-device power increase per allocator step (slow creep up).
pub const UP_RATE_W: f64 = 2.0;
/// Max per-device power decrease per allocator step (fast back-off).
pub const DOWN_RATE_W: f64 = 8.0;
/// CPU sustained *hardware* ceiling (HX 370 cTDP max, design §1). This is the
/// absolute backstop: the operating max (`Config::cpu_max_w`) defaults to it
/// and is clamped to it. The allocator's own split uses the runtime operating
/// max `AllocInput::cpu_max_w`, NOT this const.
pub const CPU_MAX_W: f64 = 54.0;
/// GPU *hardware* ceiling (RTX 5070 module AC TGP, design §1). Same role as
/// [`CPU_MAX_W`]: the clamp bound / default for `Config::gpu_max_w`, not the
/// live split max (that is `AllocInput::gpu_max_w`).
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
        Some(limit)
            if limit.is_finite() && limit > 0.0 && s.cpu_pkg_w.is_finite() && s.cpu_pkg_w > 0.0 =>
        {
            if limit - s.cpu_pkg_w <= CPU_PINNED_MARGIN_W {
                1.0
            } else {
                (s.cpu_pkg_w / limit).min(1.0)
            }
        }
        _ => util_frac(s.cpu_util_pct),
    };

    let gpu_pinned = gpu_max_mhz.is_some_and(|max| {
        s.gpu_mhz_valid
            && s.gpu_sm_mhz.is_finite()
            && f64::from(max) - s.gpu_sm_mhz <= GPU_PINNED_MARGIN_MHZ
            && s.gpu_util_pct > GPU_PINNED_UTIL_PCT
    });
    let gpu_starved = if gpu_pinned {
        1.0
    } else {
        match gpu_target_w {
            Some(t) if t.is_finite() && t > 0.0 && s.gpu_w_valid && s.gpu_w.is_finite() => {
                (s.gpu_w / t).clamp(0.0, 1.0)
            }
            _ => util_frac(s.gpu_util_pct),
        }
    };

    Demand {
        cpu_starved,
        gpu_starved,
    }
}

/// `pct/100` clamped to 0..=1; non-finite (garbage sensor math) → 0 — never
/// raise power off a reading that is not a number.
fn util_frac(pct: f64) -> f64 {
    if pct.is_finite() {
        (pct / 100.0).clamp(0.0, 1.0)
    } else {
        0.0
    }
}

/// One allocator step's inputs (design §3.1).
#[derive(Debug, Clone, Copy)]
pub struct AllocInput {
    /// Total power budget (W) for this step, from the single integrator
    /// (design §2.4, `control/budget.rs`). Until that integrator is wired
    /// (`fw-fanctrl-loop-j6s`), callers pass a placeholder.
    pub budget_w: f64,
    pub demand: Demand,
    /// CPU floor (W): `split_budget` never allocates the CPU axis below this.
    pub floors: f64,
    /// Operating power ceiling (W) for the CPU axis this step: the
    /// config-driven `Config::cpu_max_w`, already clamped to the hardware
    /// ceiling [`CPU_MAX_W`].
    pub cpu_max_w: f64,
    /// Operating power ceiling (W) for the GPU axis this step: the
    /// config-driven `Config::gpu_max_w`, already clamped to the hardware
    /// ceiling [`GPU_MAX_W`].
    pub gpu_max_w: f64,
    /// GPU floor (W): `split_budget` never allocates the GPU axis below
    /// this. Design §2.4: the LUT's watts at the configured GPU clock floor
    /// — the allocator itself no longer knows about clocks or the LUT, so
    /// the caller converts.
    pub gpu_floor_w: f64,
}

/// Splits `budget_w` between the CPU and GPU axes (design §3.1):
/// 1. Both floors are met first, regardless of how small `budget_w` is
///    (even a budget below the floors' sum still returns both floors).
/// 2. Whatever remains above the floors' sum is split in proportion to
///    `demand` (`cpu_starved`, `gpu_starved`); when both scores are zero
///    the remainder splits evenly.
/// 3. Each axis is capped at its own max; any surplus an axis cannot absorb
///    there is handed to the other axis (which may itself already be at its
///    cap — the surplus is then simply unspent, since `budget_w` should
///    never exceed `cpu_max_w + gpu_max_w` in practice).
///
/// Pure function: no rate limiting, no grid quantisation — both need the
/// previous commanded point, which only [`Allocator::step`] holds.
pub fn split_budget(
    budget_w: f64,
    demand: Demand,
    cpu_floor_w: f64,
    gpu_floor_w: f64,
    cpu_max_w: f64,
    gpu_max_w: f64,
) -> (f64, f64) {
    let remainder = (budget_w - cpu_floor_w - gpu_floor_w).max(0.0);

    let starved_sum = demand.cpu_starved + demand.gpu_starved;
    let (cpu_frac, gpu_frac) = if starved_sum > 0.0 {
        (
            demand.cpu_starved / starved_sum,
            demand.gpu_starved / starved_sum,
        )
    } else {
        (0.5, 0.5)
    };

    let cpu_raw = cpu_floor_w + remainder * cpu_frac;
    let gpu_raw = gpu_floor_w + remainder * gpu_frac;

    let cpu_capped = cpu_raw.min(cpu_max_w);
    let gpu_capped = gpu_raw.min(gpu_max_w);
    // What each axis's own cap rejected, handed to the other axis. If the
    // other axis is itself already at its cap, the second `.min` below just
    // drops the unspent remainder rather than reassigning it further — a
    // two-axis split has nowhere else to put it.
    let cpu_over = cpu_raw - cpu_capped;
    let gpu_over = gpu_raw - gpu_capped;

    let cpu_w = (cpu_capped + gpu_over).min(cpu_max_w).max(cpu_floor_w);
    let gpu_w = (gpu_capped + cpu_over).min(gpu_max_w).max(gpu_floor_w);

    (cpu_w, gpu_w)
}

/// Rounds a watts value to the nearest [`GRID_STEP_W`] grid point.
fn quantize(w: f64) -> f64 {
    (w / GRID_STEP_W).round() * GRID_STEP_W
}

/// Turns a scalar power budget into a per-device allocation, honoring the
/// demand split, both floors and the asymmetric slew clamp. Pure policy: no
/// hardware.
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
    /// 1. [`split_budget`] turns `inp.budget_w` into a per-axis raw target,
    ///    floors met first, demand-proportional above them, capped per axis
    ///    with surplus reassigned to the other.
    /// 2. Each axis's per-tick change from the previous commanded point is
    ///    bounded by [`UP_RATE_W`] / [`DOWN_RATE_W`] (asymmetric: slow
    ///    creep up, fast back-off).
    /// 3. The rate-clamped value is quantised to [`GRID_STEP_W`], then
    ///    re-clamped into the same rate bound: quantising can itself walk
    ///    the value up to half a grid step past the bound just enforced in
    ///    step 2, so it is pulled back inside before anything else touches
    ///    it.
    /// 4. The (possibly just-raised) floor wins over both the slew clamp and
    ///    the quantisation: it is re-applied last, *unquantised*, so a
    ///    raised floor lifts its axis on this very step rather than waiting
    ///    out [`UP_RATE_W`], and a floor that is not itself grid-aligned
    ///    (e.g. `cpu_floor_w = 15.2`) is still met exactly rather than
    ///    getting rounded back down below itself.
    pub fn step(&mut self, inp: &AllocInput) -> (f64, f64) {
        debug_assert!(
            (0.0..=inp.cpu_max_w).contains(&inp.floors),
            "cpu floor {} outside [0, {}]",
            inp.floors,
            inp.cpu_max_w
        );
        debug_assert!(
            (0.0..=inp.gpu_max_w).contains(&inp.gpu_floor_w),
            "gpu floor {} outside [0, {}]",
            inp.gpu_floor_w,
            inp.gpu_max_w
        );
        let cpu_floor = inp.floors.clamp(0.0, inp.cpu_max_w);
        let gpu_floor = inp.gpu_floor_w.clamp(0.0, inp.gpu_max_w);
        let prev = self.last.unwrap_or((cpu_floor, gpu_floor));

        let (raw_cpu, raw_gpu) = split_budget(
            inp.budget_w,
            inp.demand,
            cpu_floor,
            gpu_floor,
            inp.cpu_max_w,
            inp.gpu_max_w,
        );

        // Quantize the rate-clamped value, then re-clamp the quantized
        // result back into the same rate bound before finally re-applying
        // the floor *without* re-quantizing.
        //
        // The re-clamp matters because rounding to the nearest GRID_STEP_W
        // can itself walk the value up to half a grid step *outside* the
        // bound that was just enforced (e.g. a rate-clamped 17.751 quantizes
        // to 18.0, which is 0.249 W past a +2.0 W up-rate from a non-grid
        // previous point like 15.751 — `gpu_floor_w` is an LUT-interpolated
        // wattage and thus essentially always off-grid, and `cpu_floor_w` is
        // a raw config value never grid-aligned by `Config::sanitized`, so
        // this was not a rare corner case). Clamping again after quantizing
        // pulls that drift back inside UP_RATE_W/DOWN_RATE_W.
        //
        // The floor is re-applied last, and unquantized, and *after* the
        // second rate clamp — not folded into it — because the floor is a
        // hard safety bound that must win even over the rate limit itself
        // (a floor raised between steps has to lift its axis immediately,
        // not wait out UP_RATE_W), and quantizing after the floor clamp can
        // round the result back down below a floor that is not itself a
        // multiple of GRID_STEP_W (e.g. a configured cpu_floor_w of 15.2),
        // silently violating the "floors always met" invariant.
        let cpu_w = quantize(raw_cpu.clamp(prev.0 - DOWN_RATE_W, prev.0 + UP_RATE_W))
            .clamp(prev.0 - DOWN_RATE_W, prev.0 + UP_RATE_W)
            .max(cpu_floor);
        let gpu_w = quantize(raw_gpu.clamp(prev.1 - DOWN_RATE_W, prev.1 + UP_RATE_W))
            .clamp(prev.1 - DOWN_RATE_W, prev.1 + UP_RATE_W)
            .max(gpu_floor);

        let out = (cpu_w, gpu_w);
        self.last = Some(out);
        out
    }

    /// Forget history: the next step starts from the floors again. The Auto
    /// controller exits by dropping its whole loop state — equivalent to
    /// (and covered by the same tests as) this reset; kept as the explicit
    /// API for callers that hold on to an Allocator.
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.last = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const D_EQUAL: Demand = Demand {
        cpu_starved: 1.0,
        gpu_starved: 1.0,
    };
    const D_ZERO: Demand = Demand {
        cpu_starved: 0.0,
        gpu_starved: 0.0,
    };
    const D_CPU_ONLY: Demand = Demand {
        cpu_starved: 1.0,
        gpu_starved: 0.0,
    };
    const D_GPU_ONLY: Demand = Demand {
        cpu_starved: 0.0,
        gpu_starved: 1.0,
    };

    // ---- split_budget: floors ------------------------------------------

    #[test]
    fn floors_always_met_even_under_budget() {
        // Budget below the sum of the floors: both floors still come back.
        let (cpu_w, gpu_w) = split_budget(10.0, D_EQUAL, 15.0, 10.0, 54.0, 100.0);
        assert_eq!(cpu_w, 15.0);
        assert_eq!(gpu_w, 10.0);
    }

    #[test]
    fn floors_met_exactly_at_the_floors_sum() {
        let (cpu_w, gpu_w) = split_budget(25.0, D_EQUAL, 15.0, 10.0, 54.0, 100.0);
        assert_eq!(cpu_w, 15.0);
        assert_eq!(gpu_w, 10.0);
    }

    #[test]
    fn zero_floors_are_a_no_op() {
        let (cpu_w, gpu_w) = split_budget(40.0, D_EQUAL, 0.0, 0.0, 54.0, 100.0);
        assert_eq!(cpu_w, 20.0);
        assert_eq!(gpu_w, 20.0);
    }

    // ---- split_budget: demand-proportional remainder --------------------

    #[test]
    fn remainder_splits_in_proportion_to_demand() {
        // floors (5, 5), remainder 40, demand 3:1 cpu:gpu → 30/10 on top.
        let d = Demand {
            cpu_starved: 0.75,
            gpu_starved: 0.25,
        };
        let (cpu_w, gpu_w) = split_budget(50.0, d, 5.0, 5.0, 54.0, 100.0);
        assert!((cpu_w - 35.0).abs() < 1e-9, "cpu_w = {cpu_w}");
        assert!((gpu_w - 15.0).abs() < 1e-9, "gpu_w = {gpu_w}");
    }

    #[test]
    fn cpu_only_demand_sends_the_whole_remainder_to_cpu() {
        let (cpu_w, gpu_w) = split_budget(50.0, D_CPU_ONLY, 5.0, 5.0, 54.0, 100.0);
        assert_eq!(cpu_w, 45.0);
        assert_eq!(gpu_w, 5.0);
    }

    #[test]
    fn equal_split_at_zero_demand() {
        // Neither axis is starved: the surplus above the floors splits evenly.
        let (cpu_w, gpu_w) = split_budget(50.0, D_ZERO, 10.0, 10.0, 54.0, 100.0);
        assert_eq!(cpu_w, 25.0);
        assert_eq!(gpu_w, 25.0);
    }

    // ---- split_budget: caps + surplus reassignment -----------------------

    #[test]
    fn cpu_surplus_over_its_cap_is_reassigned_to_gpu() {
        // All-CPU demand wants the whole 100 W remainder on cpu_max_w = 20;
        // the 80 W it cannot hold goes to GPU instead of being dropped.
        let (cpu_w, gpu_w) = split_budget(100.0, D_CPU_ONLY, 0.0, 0.0, 20.0, 100.0);
        assert_eq!(cpu_w, 20.0, "capped at cpu_max_w");
        assert_eq!(gpu_w, 80.0, "cpu's rejected surplus lands on gpu");
    }

    #[test]
    fn gpu_surplus_over_its_cap_is_reassigned_to_cpu() {
        let (cpu_w, gpu_w) = split_budget(100.0, D_GPU_ONLY, 0.0, 0.0, 100.0, 30.0);
        assert_eq!(gpu_w, 30.0, "capped at gpu_max_w");
        assert_eq!(cpu_w, 70.0, "gpu's rejected surplus lands on cpu");
    }

    #[test]
    fn surplus_neither_axis_can_absorb_is_simply_unspent() {
        // Budget exceeds cpu_max_w + gpu_max_w: both axes pin at their caps,
        // the excess has nowhere to go and is dropped rather than violating
        // either cap.
        let (cpu_w, gpu_w) = split_budget(200.0, D_EQUAL, 0.0, 0.0, 20.0, 30.0);
        assert_eq!(cpu_w, 20.0);
        assert_eq!(gpu_w, 30.0);
    }

    // ---- Allocator::step: slew clamp -------------------------------------

    fn step_inp(budget_w: f64, demand: Demand) -> AllocInput {
        AllocInput {
            budget_w,
            demand,
            floors: 15.0,
            cpu_max_w: 54.0,
            gpu_max_w: 100.0,
            gpu_floor_w: 5.0,
        }
    }

    #[test]
    fn up_rate_clamps_the_first_step_from_the_floors() {
        let mut alloc = Allocator::new();
        // First step starts from the floors (15, 5); a huge budget wants far
        // more than UP_RATE_W = 2.0 W of climb in one tick.
        let (cpu_w, gpu_w) = alloc.step(&step_inp(200.0, D_EQUAL));
        assert_eq!(cpu_w, 17.0, "cpu floor + UP_RATE_W");
        assert_eq!(gpu_w, 7.0, "gpu floor + UP_RATE_W");
    }

    #[test]
    fn down_rate_clamps_a_shrinking_budget() {
        let mut alloc = Allocator::new();
        let mut prev = (0.0, 0.0);
        for _ in 0..40 {
            prev = alloc.step(&step_inp(200.0, D_EQUAL)); // climb toward the caps
        }
        let (cpu_w, gpu_w) = alloc.step(&step_inp(20.0, D_EQUAL)); // budget collapses
        // DOWN_RATE_W = 8.0: one tick cannot cut further than that, even
        // though the target (well below the floors' sum here) is far below.
        assert!(
            (prev.0 - cpu_w - DOWN_RATE_W).abs() < 1e-9,
            "cpu_w = {cpu_w} should be exactly DOWN_RATE_W below the previous point {}",
            prev.0
        );
        assert!(
            (prev.1 - gpu_w - DOWN_RATE_W).abs() < 1e-9,
            "gpu_w = {gpu_w} should be exactly DOWN_RATE_W below the previous point {}",
            prev.1
        );
    }

    #[test]
    fn per_tick_change_never_exceeds_the_rate_limits() {
        let mut alloc = Allocator::new();
        let mut prev = (15.0, 5.0); // the floors, the implicit start
        for budget in [90.0, 5.0, 160.0, 5.0, 90.0] {
            let (cpu_w, gpu_w) = alloc.step(&step_inp(budget, D_EQUAL));
            let d_cpu = cpu_w - prev.0;
            let d_gpu = gpu_w - prev.1;
            assert!(
                (-DOWN_RATE_W - 1e-9..=UP_RATE_W + 1e-9).contains(&d_cpu),
                "cpu step {d_cpu} outside [-{DOWN_RATE_W}, {UP_RATE_W}]"
            );
            assert!(
                (-DOWN_RATE_W - 1e-9..=UP_RATE_W + 1e-9).contains(&d_gpu),
                "gpu step {d_gpu} outside [-{DOWN_RATE_W}, {UP_RATE_W}]"
            );
            prev = (cpu_w, gpu_w);
        }
    }

    // ---- Allocator::step: floor wins over the slew clamp -----------------

    #[test]
    fn a_raised_cpu_floor_lifts_the_axis_past_the_up_rate() {
        let mut alloc = Allocator::new();
        alloc.step(&step_inp(20.0, D_EQUAL)); // settle near the floors
        let mut inp = step_inp(20.0, D_EQUAL);
        inp.floors = 40.0; // floor jumps 25 W above the last commanded point
        let (cpu_w, _) = alloc.step(&inp);
        assert_eq!(cpu_w, 40.0, "the raised floor must land immediately");
    }

    #[test]
    fn a_raised_gpu_floor_lifts_the_axis_past_the_up_rate() {
        let mut alloc = Allocator::new();
        alloc.step(&step_inp(20.0, D_EQUAL));
        let mut inp = step_inp(20.0, D_EQUAL);
        inp.gpu_floor_w = 60.0;
        let (_, gpu_w) = alloc.step(&inp);
        assert_eq!(gpu_w, 60.0, "the raised gpu floor must land immediately");
    }

    // ---- Allocator::step: floor beats grid quantisation -------------------

    #[test]
    fn a_non_grid_aligned_cpu_floor_is_met_exactly_through_step() {
        // cpu_floor_w = 15.2 is a legal config value (Config::validate only
        // clamps to [0, cpu_max_w], it never grid-aligns) but is not a
        // multiple of GRID_STEP_W = 0.5. Regression: quantizing the
        // floor-clamped value used to round 15.2 down to 15.0, violating the
        // "floors always met" invariant. The floor must come out exactly,
        // even though that leaves the output off the 0.5 W grid.
        let mut alloc = Allocator::new();
        let inp = AllocInput {
            budget_w: 15.2, // right at the floor: raw_cpu == cpu_floor == 15.2
            demand: D_EQUAL,
            floors: 15.2,
            cpu_max_w: 54.0,
            gpu_max_w: 100.0,
            gpu_floor_w: 5.0,
        };
        let (cpu_w, _) = alloc.step(&inp);
        assert_eq!(
            cpu_w, 15.2,
            "cpu floor must be met exactly, not quantised down"
        );
    }

    #[test]
    fn a_non_grid_aligned_gpu_floor_is_met_exactly_through_step() {
        // Symmetric case on the GPU axis (gpu_floor_w derives from an
        // interpolated LUT lookup in the real controller, so it is routinely
        // off-grid too). quantize(7.2) rounds down to 7.0 — budget is set to
        // exactly the floors' sum so both raw targets land exactly on their
        // floor (no rate-clamp interference), isolating the quantise-vs-floor
        // ordering this regression is about.
        let mut alloc = Allocator::new();
        let inp = AllocInput {
            budget_w: 22.2, // == cpu floor (15.0) + gpu floor (7.2): remainder is 0
            demand: D_EQUAL,
            floors: 15.0,
            cpu_max_w: 54.0,
            gpu_max_w: 100.0,
            gpu_floor_w: 7.2,
        };
        let (_, gpu_w) = alloc.step(&inp);
        assert_eq!(
            gpu_w, 7.2,
            "gpu floor must be met exactly, not quantised down"
        );
    }

    #[test]
    fn quantising_a_non_grid_prev_point_does_not_blow_the_up_rate() {
        // Regression: quantising the rate-clamped value can itself round the
        // result *outside* the bound the rate clamp just enforced, when the
        // previous commanded point is off-grid. That happens whenever a
        // floor *rise* outruns UP_RATE_W: the rate clamp caps the climb
        // below the new floor, so the "floor always wins" override sets the
        // commanded point to the floor exactly — off-grid, whatever the
        // floor's own alignment (gpu_floor_w is always an LUT-interpolated
        // wattage, and cpu_floor_w is never grid-aligned by
        // `Config::sanitized`, so this is routine, not a rare corner case).
        // The next tick that wants to climb further then saturates the rate
        // clamp again, and pre-fix, quantize(prev + UP_RATE_W) rounded up
        // past prev + UP_RATE_W by up to half a grid step (e.g. prev =
        // 15.751 -> bound 17.751 -> quantize 18.0: a 2.249 W delta against
        // the 2.0 W cap).
        let mut alloc = Allocator::new();

        // Step 1: seed both axes at a grid-aligned (0, 0) start.
        let (cpu0, gpu0) = alloc.step(&AllocInput {
            budget_w: 0.0,
            demand: D_EQUAL,
            floors: 0.0,
            cpu_max_w: 54.0,
            gpu_max_w: 100.0,
            gpu_floor_w: 0.0,
        });
        assert_eq!((cpu0, gpu0), (0.0, 0.0), "premise: seeded at the floors");

        // Step 2: both floors jump up by more than UP_RATE_W in one tick
        // (15.751 and 8.251 are both legal, non-grid-aligned values). The
        // rate clamp caps the raw climb at 0 + UP_RATE_W = 2.0, well below
        // either new floor, so the floor override must fire and land each
        // axis exactly on its (off-grid) floor.
        let (cpu1, gpu1) = alloc.step(&AllocInput {
            budget_w: 15.751 + 8.251, // remainder 0: raw == floor on each axis
            demand: D_EQUAL,
            floors: 15.751,
            cpu_max_w: 54.0,
            gpu_max_w: 100.0,
            gpu_floor_w: 8.251,
        });
        assert_eq!(
            (cpu1, gpu1),
            (15.751, 8.251),
            "premise: a floor rise that outruns UP_RATE_W lands exactly on the (off-grid) floor"
        );

        // Step 3: floors hold, but demand is large enough to saturate the
        // up-rate clamp on both axes from this off-grid previous point.
        let (cpu2, gpu2) = alloc.step(&AllocInput {
            budget_w: 100.0,
            demand: D_EQUAL,
            floors: 15.751,
            cpu_max_w: 54.0,
            gpu_max_w: 100.0,
            gpu_floor_w: 8.251,
        });
        assert!(
            cpu2 - cpu1 <= UP_RATE_W + 1e-9,
            "cpu step-3 delta {} exceeds UP_RATE_W {} from non-grid prev {}",
            cpu2 - cpu1,
            UP_RATE_W,
            cpu1
        );
        assert!(
            gpu2 - gpu1 <= UP_RATE_W + 1e-9,
            "gpu step-3 delta {} exceeds UP_RATE_W {} from non-grid prev {}",
            gpu2 - gpu1,
            UP_RATE_W,
            gpu1
        );
        // Exact expected values: quantize(prev + UP_RATE_W) re-clamped back
        // down to prev + UP_RATE_W (18.0 -> 17.751, 10.5 -> 10.251).
        assert_eq!(cpu2, cpu1 + UP_RATE_W);
        assert_eq!(gpu2, gpu1 + UP_RATE_W);
    }

    // ---- Allocator::step: grid quantisation -------------------------------

    #[test]
    fn output_is_quantised_to_the_grid_step() {
        let mut alloc = Allocator::new();
        // From a (0, 0) floor start the up-rate clamp bounds the first step
        // to [0, UP_RATE_W] = [0, 2.0], so pick a budget whose demand-even
        // split (1.26 W/axis) lands off-grid but stays inside that bound.
        let inp = AllocInput {
            budget_w: 2.52,
            demand: D_EQUAL,
            floors: 0.0,
            cpu_max_w: 54.0,
            gpu_max_w: 100.0,
            gpu_floor_w: 0.0,
        };
        let (cpu_w, gpu_w) = alloc.step(&inp);
        assert_eq!(cpu_w, 1.5, "1.26 rounds to the nearest 0.5 W grid point");
        assert_eq!(gpu_w, 1.5, "1.26 rounds to the nearest 0.5 W grid point");
    }

    #[test]
    fn quantisation_rounds_to_the_nearer_grid_point() {
        assert_eq!(quantize(10.24), 10.0);
        assert_eq!(quantize(10.26), 10.5);
        assert_eq!(quantize(10.25), 10.5); // .round() rounds half away from zero
    }

    // ---- Allocator::step: reset --------------------------------------

    #[test]
    fn reset_restarts_from_the_floors() {
        let mut alloc = Allocator::new();
        alloc.step(&step_inp(200.0, D_EQUAL));
        alloc.reset();
        let (cpu_w, gpu_w) = alloc.step(&step_inp(200.0, D_EQUAL));
        // Same as the very first step in `up_rate_clamps_the_first_step_from_the_floors`.
        assert_eq!(cpu_w, 17.0);
        assert_eq!(gpu_w, 7.0);
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

    #[test]
    fn demand_non_finite_readings_score_zero() {
        let mut s = sample();
        s.cpu_pkg_w = f64::NAN;
        s.cpu_util_pct = f64::INFINITY;
        s.gpu_w = f64::NAN;
        s.gpu_util_pct = f64::NAN;
        s.gpu_sm_mhz = f64::INFINITY; // would look "clock-pinned" unguarded
        let d = demand(&s, Some(40.0), Some(80.0), Some(2000));
        assert_eq!(d.cpu_starved, 0.0, "garbage reading must not raise power");
        assert_eq!(d.gpu_starved, 0.0, "garbage reading must not raise power");
    }
}

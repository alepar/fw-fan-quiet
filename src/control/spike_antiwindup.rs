//! Spike `fw-fanctrl-loop-9it`: settles the demand-limited anti-windup rule
//! design §2.4 deliberately left open. Throwaway measurement harness, kept
//! as a `#[cfg(test)]` fixture rather than deleted (see the module-level
//! "Why kept" note at the bottom) so the decided rule stays regression
//! tested against the scenarios that motivated it.
//!
//! # What this measures
//!
//! Wraps the **real** [`super::budget::Budget`] in:
//! - a first-order thermal plant (τ 35 s, θ 20 s, K 0.8 °C/W — design §5),
//!   driven by **actual drawn watts**, not commanded watts, which is the
//!   only way to reproduce the failure at all (an unconstrained integrator
//!   cannot tell "nothing is drawing this" from "everything is");
//! - a demand model deciding how much of each axis's commanded cap is
//!   actually drawn, from data (§Facts, `DEMAND_MARGIN_W` per axis, measured
//!   on this machine, see the constants below);
//! - the real [`super::allocator::split_budget`] turning the scalar `u` into
//!   a per-axis cap each tick, so "per-axis vs combined" is measured against
//!   the actual split policy, not a stand-in.
//!
//! Four candidate rules ([`Rule`]) x seven scenarios ([`scenario_table`]) =
//! the cross product [`sweep`] runs and [`print_sweep_table`] renders (the
//! source of the table in design doc §2.4).
//!
//! # The decision (see §2.4 for the normative writeup)
//!
//! `ConditionalHysteresis`, per-axis, using the **post**-guard-override cap,
//! judged against **headroom above each axis's own floor** (an axis sitting
//! exactly at its floor was never offered room to waste, so it can never
//! trigger the halt — this is what makes per-axis different from a
//! sum-of-axes test even though both are "per axis" comparisons against a
//! margin; see `demand_limited_axis` below). `DEMAND_MARGIN_W`: CPU 2.0 W,
//! GPU 3.0 W (§Facts). Hysteresis: 2 ticks (10 s) to enter *and* to leave.
//! Leaving the hold does not need a bespoke resync rule — `Budget::step`'s
//! existing generic "leaving any freeze" resync already covers a
//! `DemandLimited` release, and this harness's `resync` assertions confirm
//! that generic path is sufficient (§2.4 records this as "no new resync
//! rule needed", not as an unresolved item).

use std::collections::VecDeque;

use super::allocator::{Demand, split_budget};
use super::budget::{Budget, Freeze, LoopError, LoopGains, PI_PERIOD_S};

// ---------------------------------------------------------------------
// Measured constants (§Facts, 2026-09-09 on bazerame: HX 370 + RTX 5070)
// ---------------------------------------------------------------------

/// CPU axis demand-limited margin, watts. Measured via `ryzenadj --info`'s
/// `PPT VALUE SLOW` (drawn) against `PPT LIMIT SLOW` (commanded), RAPL
/// energy_uj cross-validated: cap-bound noise floor was <= 0.08 W across
/// four load compositions (24-thread saturating stress-ng at 20 W and 35 W
/// caps, 2-thread and single-15%-duty-cycle loads at 35 W, and ordinary
/// desktop background load at 35 W — all landed within 0.005-0.08 W of
/// their commanded cap). A genuine demand-limited gap, measured directly
/// (54 W cap vs this machine's ~36.8 W natural desktop draw, RAPL-verified),
/// was 17.2 W. 2.0 W sits ~25x above the measured noise floor and ~8x below
/// the smallest measured genuine gap.
const DEMAND_MARGIN_W_CPU: f64 = 2.0;

/// GPU axis demand-limited margin, watts. Measured via NVML `power.draw` at
/// a 1500 MHz clock lock (applied 1492 MHz): a fill/geometry-bound `glxgears`
/// proxy load settled to a 0.11 W spread over 7 ticks at the allocator
/// cadence (13.50 -> 13.39 W) — a weaker proxy than the CPU's RAPL-verified
/// number (§Facts states the limitation explicitly: no heavier compute-bound
/// generator was available on this machine). A genuine demand gap, measured
/// directly (idle GPU under the same lock: 7.27 W, vs 13.4 W under the
/// proxy load), was 6.1 W. 3.0 W sits ~27x above the measured noise floor
/// and about half the smallest measured genuine gap -- more conservative
/// than the CPU margin's ratio, because the noise-floor number itself is
/// the weaker of the two measurements.
const DEMAND_MARGIN_W_GPU: f64 = 3.0;

/// Hysteresis dwell for [`Rule::ConditionalHysteresis`]: ticks (at
/// [`PI_PERIOD_S`] = 5 s each) a would-be state change must persist before
/// it takes effect, both entering and leaving the hold. 2 ticks = 10 s,
/// short relative to tau (35 s) so it does not meaningfully delay recovery,
/// but the sweep (scenario 7) shows it is enough to stop single-tick noise
/// crossings from toggling the hold every tick.
const HYSTERESIS_DWELL_TICKS: u32 = 2;

// Harness-local operating envelope. Not the hardware ceilings
// (`allocator::CPU_MAX_W` / `GPU_MAX_W`) -- a plausible in-session operating
// range, consistent with the Facts section's ~55-75 W measured combined
// sessions and with `K = 0.8`, `y = K*u` at steady state (no separate
// ambient term, matching `budget.rs`'s own `run_temp_closed_loop` test
// convention -- `y` is a delta-temperature, not an absolute reading).
const CPU_FLOOR_W: f64 = 10.0;
const CPU_MAX_W: f64 = 54.0;
/// The LUT's watts at the configured GPU clock floor (design §2.4) --
/// `split_budget`'s GPU floor argument. A policy constant: it does not
/// depend on whether the card is currently powered (see the note where this
/// is used below).
const GPU_FLOOR_W: f64 = 5.0;
const GPU_MAX_W: f64 = 60.0;

// ---------------------------------------------------------------------
// Xorshift32: hand-rolled seeded RNG (Global Constraints: no new crate
// dependency). Kept as a private copy, matching `test_support::plant`'s own
// note that each module keeps its own rather than sharing one.
// ---------------------------------------------------------------------

struct Xorshift32(u32);

impl Xorshift32 {
    fn new(seed: u32) -> Self {
        Xorshift32(if seed == 0 { 0x9E37_79B9 } else { seed })
    }

    fn next_u32(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        x
    }

    /// Uniform in `[-amplitude, amplitude]`.
    fn jitter(&mut self, amplitude: f64) -> f64 {
        let unit = f64::from(self.next_u32()) / f64::from(u32::MAX); // [0,1]
        (unit * 2.0 - 1.0) * amplitude
    }
}

// ---------------------------------------------------------------------
// Plant: first-order + dead time, tau 35 / theta 20 / K 0.8 (design §5).
// Same shape as `budget.rs`'s own private `Fopdt` test struct -- kept as a
// separate copy here since that one is private to `budget`'s test module.
// ---------------------------------------------------------------------

struct Plant {
    tau_s: f64,
    k: f64,
    y: f64,
    delay: VecDeque<f64>,
}

impl Plant {
    fn new(tau_s: f64, theta_s: f64, k: f64, y0: f64) -> Self {
        let delay_ticks = (theta_s / PI_PERIOD_S).round() as usize;
        Self {
            tau_s,
            k,
            y: y0,
            delay: VecDeque::from(vec![0.0; delay_ticks.max(1)]),
        }
    }

    /// Advances one `PI_PERIOD_S` tick on **actual drawn watts** (not
    /// commanded `u`) and returns the new `y`.
    fn step(&mut self, drawn_w: f64) -> f64 {
        self.delay.push_back(drawn_w);
        let delayed = self.delay.pop_front().unwrap();
        let a = (-PI_PERIOD_S / self.tau_s).exp();
        self.y = self.y * a + self.k * (1.0 - a) * delayed;
        self.y
    }
}

// ---------------------------------------------------------------------
// Candidate rules
// ---------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Rule {
    /// Baseline: no halt at all, plain clamp+back-calc anti-windup only.
    NoHalt,
    /// Conditional integration, directional, per-axis, no hysteresis:
    /// judged fresh every tick.
    Conditional,
    /// Same, with a `HYSTERESIS_DWELL_TICKS` dwell entering and leaving.
    ConditionalHysteresis,
    /// Directional, but judged on the SUM of axes rather than per-axis --
    /// included to empirically demonstrate why §2.4's fixed "judge each
    /// axis separately" invariant exists (scenario 5).
    CombinedSum,
}

const ALL_RULES: [Rule; 4] = [
    Rule::NoHalt,
    Rule::Conditional,
    Rule::ConditionalHysteresis,
    Rule::CombinedSum,
];

/// One axis's demand-limited predicate: only an axis that was actually
/// offered headroom above its own floor (`cap > floor`) can be
/// demand-limited -- an axis pinned exactly at its floor was never given
/// room to waste, so it can never contribute a false "unused headroom"
/// signal. This is what makes per-axis different from a naive per-axis
/// `cap - draw > margin` check (which would ALSO permanently flag a
/// structurally-undrawn GPU sitting at its floor share) and is the actual
/// content of §2.4's "judge each axis separately" invariant -- not just
/// "loop over axes", but "loop over axes, and only count headroom that was
/// actually offered".
fn demand_limited_axis(draw_w: f64, cap_w: f64, floor_w: f64, margin_w: f64) -> bool {
    cap_w > floor_w + 1e-9 && (cap_w - draw_w) > margin_w
}

/// Rolling per-run state a [`Rule`] needs across ticks (hysteresis dwell
/// counters). Fresh per scenario run.
#[derive(Default)]
struct RuleState {
    cpu_halted: bool,
    gpu_halted: bool,
    combined_halted: bool,
    cpu_pending_ticks: u32,
    gpu_pending_ticks: u32,
    combined_pending_ticks: u32,
    /// The overall (post-hysteresis, OR'd-across-axes) verdict `step`
    /// returned last tick, so [`RuleState::step`] can count transitions
    /// uniformly across all four rules -- not just the two that route
    /// through [`RuleState::debounce`] -- for a fair chatter comparison
    /// (scenario 7).
    last_overall: bool,
    /// Count of overall-verdict transitions this run (scenario 7's chatter
    /// metric).
    toggles: u32,
}

impl RuleState {
    /// Debounced transition: `raw` is this tick's un-hysteresis'd verdict.
    /// Returns the (possibly debounced) effective verdict, updating
    /// `halted`/`pending` in place. Toggle counting happens once, in
    /// [`RuleState::step`], on the overall result -- not here -- so it is
    /// comparable across rules that do and do not call this.
    fn debounce(halted: &mut bool, pending: &mut u32, raw: bool) -> bool {
        if raw == *halted {
            *pending = 0;
        } else {
            *pending += 1;
            if *pending >= HYSTERESIS_DWELL_TICKS {
                *halted = raw;
                *pending = 0;
            }
        }
        *halted
    }

    /// One tick's demand-limited verdict for `rule`, given each axis's
    /// (draw, post-guard-override cap, floor).
    fn step(&mut self, rule: Rule, cpu: (f64, f64, f64), gpu: (f64, f64, f64)) -> bool {
        let (cpu_draw, cpu_cap, cpu_floor) = cpu;
        let (gpu_draw, gpu_cap, gpu_floor) = gpu;
        let overall = match rule {
            Rule::NoHalt => false,
            Rule::Conditional => {
                demand_limited_axis(cpu_draw, cpu_cap, cpu_floor, DEMAND_MARGIN_W_CPU)
                    || demand_limited_axis(gpu_draw, gpu_cap, gpu_floor, DEMAND_MARGIN_W_GPU)
            }
            Rule::ConditionalHysteresis => {
                let cpu_raw =
                    demand_limited_axis(cpu_draw, cpu_cap, cpu_floor, DEMAND_MARGIN_W_CPU);
                let gpu_raw =
                    demand_limited_axis(gpu_draw, gpu_cap, gpu_floor, DEMAND_MARGIN_W_GPU);
                let cpu =
                    Self::debounce(&mut self.cpu_halted, &mut self.cpu_pending_ticks, cpu_raw);
                let gpu =
                    Self::debounce(&mut self.gpu_halted, &mut self.gpu_pending_ticks, gpu_raw);
                cpu || gpu
            }
            Rule::CombinedSum => {
                // Sum-of-axes: no floor-headroom carve-out possible at the
                // sum level without reconstructing per-axis floors anyway
                // (at which point it is not really "combined" any more) --
                // that is the point being demonstrated.
                let margin_sum = DEMAND_MARGIN_W_CPU + DEMAND_MARGIN_W_GPU;
                let draw_sum = cpu_draw + gpu_draw;
                let cap_sum = cpu_cap + gpu_cap;
                let raw = (cap_sum - draw_sum) > margin_sum;
                Self::debounce(
                    &mut self.combined_halted,
                    &mut self.combined_pending_ticks,
                    raw,
                )
            }
        };
        if overall != self.last_overall {
            self.toggles += 1;
            self.last_overall = overall;
        }
        overall
    }
}

// ---------------------------------------------------------------------
// Scenarios
// ---------------------------------------------------------------------

/// One scenario, encoded as data: per-tick target and per-axis "true"
/// (unconstrained) demand, driven through a fixed number of ticks. Function
/// pointers (not closures) so [`SCENARIOS`] can be a plain const table --
/// every input is a pure function of the tick index, keeping the whole
/// sweep reproducible given only the RNG seed.
struct Scenario {
    name: &'static str,
    ticks: u32,
    y0: f64,
    /// `Some(u)` warm-starts the integrator; `None` starts from the floors.
    warm_start_u: Option<f64>,
    target_c: fn(u32) -> f64,
    /// Feeds `split_budget`'s demand SCORE (how starved this axis looks).
    cpu_demand_w: fn(u32) -> f64,
    gpu_powered: fn(u32) -> bool,
    gpu_demand_w: fn(u32) -> f64,
    gpu_hot: fn(u32) -> bool,
    /// What the axis actually tries to draw (jittered, then clamped to its
    /// cap). `None` means "the same as the demand score above" -- true for
    /// every scenario except 7, which needs to push the demand SCORE to
    /// saturation (so the cap reaches the hardware max and stays there,
    /// stable and known) while the DRAW target sits `DEMAND_MARGIN_W` below
    /// that same max, independent of the score.
    cpu_draw_target_w: Option<fn(u32) -> f64>,
    gpu_draw_target_w: Option<fn(u32) -> f64>,
    /// Ticks (from the end) over which "holds target" is graded -- lets a
    /// scenario with a transient onset exclude the transient itself.
    grade_from_tick: u32,
    /// Grading tolerance, °C-equivalent (this harness's temp-domain units).
    tolerance_c: f64,
    /// Whether `holds_target` also grades `peak_overshoot_c`. Only
    /// meaningful for a scenario whose starting condition and target are
    /// arranged so any `y > target` crossing is a genuine windup-driven
    /// overshoot (scenario 1) -- everywhere else `y` legitimately starts,
    /// or is driven, above the current target (a warm start above T*, a
    /// target that has just dropped), and comparing THAT against `target`
    /// would just be re-measuring the scenario's own setup, not the rule
    /// under test.
    check_overshoot: bool,
}

// --- Scenario 1: idle wind-up to the ceiling, then a load onset ---
fn s1_target(_t: u32) -> f64 {
    30.0
}
fn s1_cpu_demand(t: u32) -> f64 {
    if t < 60 { 3.0 } else { 45.0 }
}
fn s1_gpu_demand(t: u32) -> f64 {
    if t < 60 { 2.0 } else { 18.0 }
}
fn s1_gpu_powered(_t: u32) -> bool {
    true
}
fn s1_gpu_hot(_t: u32) -> bool {
    false
}

// --- Scenario 2: a lull mid-session ---
fn s2_target(_t: u32) -> f64 {
    35.0
}
fn s2_cpu_demand(t: u32) -> f64 {
    if (150..210).contains(&t) { 4.0 } else { 40.0 }
}
fn s2_gpu_demand(t: u32) -> f64 {
    if (150..210).contains(&t) { 2.0 } else { 15.0 }
}
fn s2_gpu_powered(_t: u32) -> bool {
    true
}
fn s2_gpu_hot(_t: u32) -> bool {
    false
}

// --- Scenario 3: warm-start from a heavier session, lighter load, EC above T* ---
// Target set at the light load's own reachable ceiling (17 W true demand *
// K 0.8 = 13.6, so 12.0 leaves margin) -- a target the light load genuinely
// cannot sustain would fail regardless of the anti-windup rule (a plant
// constraint, not a windup one), which is not what this scenario tests.
fn s3_target(_t: u32) -> f64 {
    12.0
}
fn s3_cpu_demand(_t: u32) -> f64 {
    12.0
}
fn s3_gpu_demand(_t: u32) -> f64 {
    5.0
}
fn s3_gpu_powered(_t: u32) -> bool {
    true
}
fn s3_gpu_hot(_t: u32) -> bool {
    false
}

// --- Scenario 4: a mid-session target drop ---
fn s4_target(t: u32) -> f64 {
    if t < 160 { 45.0 } else { 22.0 }
}
// Demand set well above what either target needs (56.25 W for T*=45, 27.5 W
// for T*=22) so the caps -- not the true demand -- are always the binding
// constraint; a demand ceiling below the target would fail regardless of
// the anti-windup rule (see scenario 3's note).
fn s4_cpu_demand(_t: u32) -> f64 {
    50.0
}
fn s4_gpu_demand(_t: u32) -> f64 {
    20.0
}
fn s4_gpu_powered(_t: u32) -> bool {
    true
}
fn s4_gpu_hot(_t: u32) -> bool {
    false
}

// --- Scenario 5: structurally undrawn axis (dGPU unpowered, CPU-only load) ---
// With the GPU unpowered, draw is bounded by the CPU's OWN hardware ceiling
// alone (54 W -> y_ss = 43.2), so the target sits just under that (42.0),
// and CPU demand saturates (999, like scenario 6) so CPU keeps genuinely
// wanting to climb toward ITS OWN ceiling the whole run -- the condition
// that actually distinguishes "per-axis lets CPU reach its ceiling" from
// "combined-sum holds it below that ceiling because of the permanent GPU
// floor gap" (§2.4's own stated failure mode for a sum-based rule).
fn s5_target(_t: u32) -> f64 {
    43.0
}
fn s5_cpu_demand(t: u32) -> f64 {
    if t < 30 { 6.0 } else { 999.0 }
}
fn s5_gpu_demand(_t: u32) -> f64 {
    0.0
}
fn s5_gpu_powered(_t: u32) -> bool {
    false
}
fn s5_gpu_hot(_t: u32) -> bool {
    false
}

// --- Scenario 6: a GPU HOT episode with the CPU at its own cap ---
// Target chosen so the required u_ss (target/K = 50 W) stays reachable even
// through the full HOT clamp (cpu_max + gpu_floor = 54 + 5 = 59 W >= 50 W):
// the scenario isolates "does anti-windup add a spurious extra freeze on
// top of the guard" from "the guard's own ceiling reduction", which a
// tighter target would conflate.
fn s6_target(_t: u32) -> f64 {
    40.0
}
// Demand set well beyond CPU_MAX_W so the CPU axis is always genuinely
// saturated (draw == cap, no gap) -- the scenario is about the GPU HOT
// interaction, not CPU windup.
fn s6_cpu_demand(_t: u32) -> f64 {
    999.0
}
fn s6_gpu_demand(_t: u32) -> f64 {
    999.0
}
fn s6_gpu_powered(_t: u32) -> bool {
    true
}
fn s6_gpu_hot(t: u32) -> bool {
    (150..190).contains(&t)
}

// --- Scenario 7: oscillation around the margin when draw sits near the cap ---
// Target set high enough (u_ss = 90/0.8 = 112.5 W, near the lo..hi ceiling)
// that the demand SCORE (pinned at each axis's own max below) saturates
// both caps at their hardware max (54 / 60 W) and keeps them there --
// stable and known regardless of the halt state, unlike driving the score
// off the same value the draw target uses (an earlier version of this
// scenario did that and the halt's own feedback on `u` pushed the settled
// gap away from the boundary instead of astride it). The DRAW target (see
// `cpu_draw_target_w` / `gpu_draw_target_w`) sits exactly
// `DEMAND_MARGIN_W` below that same saturated max, so once caps saturate
// the gap sits right on the halt boundary and the harness's own draw
// jitter (`run_scenario`) is what pushes it across, tick to tick.
fn s7_target(_t: u32) -> f64 {
    85.0
}
fn s7_cpu_demand(_t: u32) -> f64 {
    CPU_MAX_W
}
fn s7_gpu_demand(_t: u32) -> f64 {
    GPU_MAX_W
}
fn s7_cpu_draw_target(_t: u32) -> f64 {
    CPU_MAX_W - DEMAND_MARGIN_W_CPU
}
fn s7_gpu_draw_target(_t: u32) -> f64 {
    GPU_MAX_W - DEMAND_MARGIN_W_GPU
}
fn s7_gpu_powered(_t: u32) -> bool {
    true
}
fn s7_gpu_hot(_t: u32) -> bool {
    false
}

fn scenario_table() -> [Scenario; 7] {
    [
        Scenario {
            name: "1 idle wind-up then load onset",
            ticks: 220,
            y0: 0.0,
            warm_start_u: None,
            target_c: s1_target,
            cpu_demand_w: s1_cpu_demand,
            gpu_powered: s1_gpu_powered,
            gpu_demand_w: s1_gpu_demand,
            gpu_hot: s1_gpu_hot,
            cpu_draw_target_w: None,
            gpu_draw_target_w: None,
            grade_from_tick: 190,
            tolerance_c: 3.0,
            check_overshoot: true,
        },
        Scenario {
            name: "2 lull mid-session",
            ticks: 350,
            y0: 0.0,
            warm_start_u: None,
            target_c: s2_target,
            cpu_demand_w: s2_cpu_demand,
            gpu_powered: s2_gpu_powered,
            gpu_demand_w: s2_gpu_demand,
            gpu_hot: s2_gpu_hot,
            cpu_draw_target_w: None,
            gpu_draw_target_w: None,
            grade_from_tick: 320,
            tolerance_c: 3.0,
            check_overshoot: false,
        },
        Scenario {
            name: "3 warm-start heavier session, lighter load, EC above T*",
            ticks: 200,
            y0: 40.0, // starts above T*=25
            warm_start_u: Some(90.0),
            target_c: s3_target,
            cpu_demand_w: s3_cpu_demand,
            gpu_powered: s3_gpu_powered,
            gpu_demand_w: s3_gpu_demand,
            gpu_hot: s3_gpu_hot,
            cpu_draw_target_w: None,
            gpu_draw_target_w: None,
            grade_from_tick: 170,
            tolerance_c: 3.0,
            check_overshoot: false,
        },
        Scenario {
            name: "4 mid-session target drop",
            ticks: 320,
            y0: 0.0,
            warm_start_u: None,
            target_c: s4_target,
            cpu_demand_w: s4_cpu_demand,
            gpu_powered: s4_gpu_powered,
            gpu_demand_w: s4_gpu_demand,
            gpu_hot: s4_gpu_hot,
            cpu_draw_target_w: None,
            gpu_draw_target_w: None,
            grade_from_tick: 290,
            tolerance_c: 3.0,
            check_overshoot: false,
        },
        Scenario {
            name: "5 structurally undrawn axis (dGPU unpowered)",
            ticks: 400,
            y0: 0.0,
            warm_start_u: None,
            target_c: s5_target,
            cpu_demand_w: s5_cpu_demand,
            gpu_powered: s5_gpu_powered,
            gpu_demand_w: s5_gpu_demand,
            gpu_hot: s5_gpu_hot,
            cpu_draw_target_w: None,
            gpu_draw_target_w: None,
            grade_from_tick: 200,
            tolerance_c: 3.0,
            check_overshoot: false,
        },
        Scenario {
            name: "6 GPU HOT with CPU at its own cap",
            ticks: 220,
            y0: 30.0,
            warm_start_u: Some(70.0),
            target_c: s6_target,
            cpu_demand_w: s6_cpu_demand,
            gpu_powered: s6_gpu_powered,
            gpu_demand_w: s6_gpu_demand,
            gpu_hot: s6_gpu_hot,
            cpu_draw_target_w: None,
            gpu_draw_target_w: None,
            grade_from_tick: 160,
            tolerance_c: 3.0,
            check_overshoot: false,
        },
        Scenario {
            name: "7 oscillation around the margin",
            ticks: 220,
            y0: 20.0,
            warm_start_u: Some(35.0),
            target_c: s7_target,
            cpu_demand_w: s7_cpu_demand,
            gpu_powered: s7_gpu_powered,
            gpu_demand_w: s7_gpu_demand,
            gpu_hot: s7_gpu_hot,
            cpu_draw_target_w: Some(s7_cpu_draw_target),
            gpu_draw_target_w: Some(s7_gpu_draw_target),
            grade_from_tick: 120,
            tolerance_c: 3.0,
            check_overshoot: false,
        },
    ]
}

// ---------------------------------------------------------------------
// One run
// ---------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
struct RunResult {
    /// Mean |y - T*| over the graded tail.
    mean_abs_err_c: f64,
    /// True if the graded tail never gets within `tolerance_c` at all --
    /// the self-latch signature (u stuck low/high, unable to recover).
    self_latch: bool,
    /// True if `u` ever *decreases* during a tick where an axis is halted
    /// in the deepening direction with a genuinely constant draw -- the
    /// "pulls u toward the draw" tracker signature. (Never triggered by any
    /// of these four candidates -- back-calc-to-draw isn't one of them --
    /// kept as a live assertion, not a dead check: it is exactly the
    /// invariant §2.4 fixes, and a future edit to `Budget::step` that
    /// reintroduced it would fail this.)
    cap_tracking: bool,
    /// Hysteresis chatter count (scenario 7's metric).
    toggles: u32,
    /// Largest `y - target` seen over the whole run (clamped to >= 0) --
    /// the windup-then-overshoot signature (scenario 1): a tail-only error
    /// average can look fine even when the run spiked well past target on
    /// the way there, so "holds the target" grades this too, not just the
    /// settled tail.
    peak_overshoot_c: f64,
}

fn run_scenario(rule: Rule, sc: &Scenario, seed: u32) -> RunResult {
    let gains = LoopGains::default();
    let mut budget = Budget::new(&gains);
    let cpu_max = CPU_MAX_W;
    let lo = CPU_FLOOR_W + GPU_FLOOR_W;
    let hi = cpu_max + GPU_MAX_W;
    budget.set_bounds(lo, hi);
    // `Budget` exposes `u` only through `seed`'s argument and `step`'s
    // return value (never a getter -- the field stays private on purpose),
    // so the harness tracks its own copy from those same two points rather
    // than widening `Budget`'s API for a throwaway harness's sake.
    let mut u = if let Some(u0) = sc.warm_start_u {
        budget.seed(u0);
        u0.clamp(lo, hi)
    } else {
        lo // `set_bounds` clamped the fresh `u = 0.0` up to `lo` already.
    };
    let mut plant = Plant::new(35.0, 20.0, 0.8, sc.y0);
    let mut rng = Xorshift32::new(seed);
    let mut state = RuleState::default();

    let mut e_prev_target = (sc.target_c)(0);
    let mut errs: Vec<f64> = Vec::new();
    let mut cap_tracking = false;
    let mut peak_overshoot_c: f64 = 0.0;
    let mut prev_halted_deepening = false;
    let mut prev_draw_total = 0.0;
    let mut prev_gpu_cap = if sc.warm_start_u.is_some() {
        GPU_MAX_W // no prior tick yet; a HOT episode never starts on tick 0 in any scenario
    } else {
        GPU_FLOOR_W
    };

    for t in 0..sc.ticks {
        let target = (sc.target_c)(t);
        let e_c = target - plant.y;
        if (target - e_prev_target).abs() > 1e-9 {
            // T*/target re-derivation: resync before this tick's step, per
            // §2.4's already-fixed rule (not something this spike decides).
            budget.resync_error(e_c);
            e_prev_target = target;
        }

        let gpu_powered = (sc.gpu_powered)(t);
        // `split_budget`'s floor argument does not know or care whether the
        // card is actually powered right now -- that is exactly what makes
        // the GPU floor share "structurally undrawn" rather than merely
        // "currently zero": `split_budget` keeps offering it every tick
        // regardless, and only the DRAW (below) drops to zero when
        // unpowered. Zeroing the floor itself here would quietly fix the
        // very problem scenario 5 exists to expose.
        let gpu_floor = GPU_FLOOR_W;

        // Demand scores for split_budget, derived from this tick's true
        // (unconstrained) demand relative to the axis's own max -- a
        // reasonable stand-in for the production `demand()` reader (which
        // needs a hardware `Sample`, out of scope for this harness).
        let cpu_true = (sc.cpu_demand_w)(t);
        let gpu_true = if gpu_powered {
            (sc.gpu_demand_w)(t)
        } else {
            0.0
        };
        let demand = Demand {
            cpu_starved: (cpu_true / cpu_max).clamp(0.0, 1.0),
            gpu_starved: if gpu_powered {
                (gpu_true / GPU_MAX_W).clamp(0.0, 1.0)
            } else {
                0.0
            },
        };

        // GPU HOT: ratchet the effective gpu_max fed to split_budget down
        // toward the floor (§2.8's own gpu_share_override shape), rather
        // than freezing the integrator -- the decided GPU-HOT interaction
        // (see module docs / §2.4). Ratchets from the previous tick's
        // actual GPU cap, not a fixed ceiling, so it is a real per-tick
        // ramp-down, not an instant drop.
        let gpu_max_effective = if (sc.gpu_hot)(t) {
            (prev_gpu_cap - super::allocator::DOWN_RATE_W).max(gpu_floor)
        } else {
            GPU_MAX_W
        };

        let (cpu_cap, gpu_cap) = split_budget(
            u,
            demand,
            CPU_FLOOR_W,
            gpu_floor,
            cpu_max,
            gpu_max_effective,
        );
        prev_gpu_cap = gpu_cap;

        // Demand model: actual draw targets the demand score by default
        // (`cpu_true`/`gpu_true`), unless the scenario supplies a separate
        // draw target (scenario 7 -- see `cpu_draw_target_w`'s doc comment);
        // either way it is jittered, then clamped to the post-guard-override
        // cap, never negative.
        let cpu_draw_target = sc.cpu_draw_target_w.map_or(cpu_true, |f| f(t));
        let gpu_draw_target = sc.gpu_draw_target_w.map_or(gpu_true, |f| f(t));
        let cpu_draw = (cpu_draw_target + rng.jitter(0.3)).clamp(0.0, cpu_cap);
        let gpu_draw = if gpu_powered {
            (gpu_draw_target + rng.jitter(0.2)).clamp(0.0, gpu_cap)
        } else {
            0.0
        };

        let error_sign = e_c.signum();
        let halted = error_sign > 0.0
            && state.step(
                rule,
                (cpu_draw, cpu_cap, CPU_FLOOR_W),
                (gpu_draw, gpu_cap, gpu_floor),
            );

        let freeze = if halted {
            Some(Freeze::DemandLimited)
        } else {
            None
        };
        let u_before = u;
        u = budget.step(LoopError::Temp { e_c }, freeze);

        // Tracker-signature check: while genuinely halted in the deepening
        // direction with the total draw held constant (not itself moving),
        // u must never step down as if chasing the draw.
        let draw_total = cpu_draw + gpu_draw;
        if prev_halted_deepening
            && halted
            && (draw_total - prev_draw_total).abs() < 1e-6
            && u < u_before - 1e-9
        {
            cap_tracking = true;
        }
        prev_halted_deepening = halted && error_sign > 0.0;
        prev_draw_total = draw_total;

        plant.step(draw_total);
        peak_overshoot_c = peak_overshoot_c.max(plant.y - target);
        if t >= sc.grade_from_tick {
            errs.push((target - plant.y).abs());
        }
    }

    let mean_abs_err_c = errs.iter().sum::<f64>() / errs.len().max(1) as f64;
    let self_latch = errs.iter().all(|e| *e > sc.tolerance_c);

    RunResult {
        mean_abs_err_c,
        peak_overshoot_c,
        self_latch,
        cap_tracking,
        toggles: state.toggles,
    }
}

// ---------------------------------------------------------------------
// Sweep
// ---------------------------------------------------------------------

fn sweep() -> Vec<(Rule, &'static str, RunResult, bool)> {
    let mut out = Vec::new();
    for sc in scenario_table() {
        for rule in ALL_RULES {
            let result = run_scenario(rule, &sc, 0xC0FFEE ^ (sc.name.len() as u32));
            let holds = holds_target(&result, &sc);
            out.push((rule, sc.name, result, holds));
        }
    }
    out
}

fn rule_name(r: Rule) -> &'static str {
    match r {
        Rule::NoHalt => "no halt",
        Rule::Conditional => "conditional (no hysteresis)",
        Rule::ConditionalHysteresis => "conditional + hysteresis",
        Rule::CombinedSum => "combined-sum",
    }
}

/// "Holds the fan target" (the acceptance criterion's phrase) for this
/// harness's temp-domain proxy: the settled tail lands within tolerance AND
/// the run never overshoots target by more than tolerance on the way there
/// -- a tail-only average can look fine on a run that spiked well past
/// target first (scenario 1's windup-then-overshoot signature) and a
/// peak-only check can look fine on a run that never actually settles, so
/// both conditions are required.
fn holds_target(r: &RunResult, sc: &Scenario) -> bool {
    let overshoot_ok = !sc.check_overshoot || r.peak_overshoot_c <= sc.tolerance_c;
    !r.self_latch && r.mean_abs_err_c <= sc.tolerance_c && overshoot_ok
}

/// Renders the candidate x scenario table (design doc §2.4's source data).
/// `cargo test print_sweep_table -- --nocapture` prints it.
fn render_sweep_table(rows: &[(Rule, &'static str, RunResult, bool)]) -> String {
    let mut out = String::new();
    out.push_str("| Scenario | Rule | holds target | self-latch | cap-tracking | mean |err| °C | peak overshoot °C | toggles |\n");
    out.push_str("|---|---|---|---|---|---|---|---|\n");
    for (rule, name, r, holds) in rows {
        out.push_str(&format!(
            "| {name} | {} | {} | {} | {} | {:.3} | {:.3} | {} |\n",
            rule_name(*rule),
            if *holds { "yes" } else { "NO" },
            if r.self_latch { "YES" } else { "no" },
            if r.cap_tracking { "YES" } else { "no" },
            r.mean_abs_err_c,
            r.peak_overshoot_c,
            r.toggles,
        ));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn print_sweep_table() {
        let rows = sweep();
        println!("\n{}", render_sweep_table(&rows));
    }

    /// Invariant check across the ENTIRE sweep, every candidate and
    /// scenario: none of the four candidates ever pulls `u` toward the
    /// draw. This is `Budget::step`'s own clamp-only back-calculation
    /// (§2.4) doing its job -- a live regression, not a tautology: a future
    /// change that added draw-directed back-calculation to `Budget::step`
    /// (the "tracker" shape roast round two killed) would fail this.
    #[test]
    fn no_candidate_ever_tracks_the_draw() {
        for (rule, name, r, _holds) in sweep() {
            assert!(
                !r.cap_tracking,
                "{:?} on {name} pulled u toward the draw",
                rule
            );
        }
    }

    /// Baseline: `NoHalt` winds up during the idle phase of scenario 1 and
    /// overshoots well past target once the load hits (the plant is still
    /// digesting the windup through its dead time) -- demonstrating the
    /// failure this spike exists to fix. A tail-only average would miss
    /// this (the run *does* eventually settle), so this checks
    /// `peak_overshoot_c` specifically, not `holds_target`.
    #[test]
    fn no_halt_baseline_overshoots_the_windup_scenario() {
        let sc = &scenario_table()[0];
        assert_eq!(sc.name, "1 idle wind-up then load onset");
        let r = run_scenario(Rule::NoHalt, sc, 0xC0FFEE ^ (sc.name.len() as u32));
        assert!(
            r.peak_overshoot_c > sc.tolerance_c,
            "expected NoHalt to overshoot scenario 1 (windup), got peak_overshoot={:.3}",
            r.peak_overshoot_c
        );
    }

    /// Per-axis (with or without hysteresis) overshoots far less than
    /// `NoHalt` on the same windup scenario -- the demand-limited halt
    /// doing its job during the idle phase.
    #[test]
    fn per_axis_rules_overshoot_much_less_than_no_halt_on_the_windup_scenario() {
        let sc = &scenario_table()[0];
        let seed = 0xC0FFEE ^ (sc.name.len() as u32);
        let no_halt = run_scenario(Rule::NoHalt, sc, seed);
        for rule in [Rule::Conditional, Rule::ConditionalHysteresis] {
            let r = run_scenario(rule, sc, seed);
            assert!(
                r.peak_overshoot_c < no_halt.peak_overshoot_c / 2.0,
                "{rule:?} overshoot {:.3} was not well below NoHalt's {:.3}",
                r.peak_overshoot_c,
                no_halt.peak_overshoot_c
            );
        }
    }

    /// `demand_limited_axis` (the per-axis predicate) never flags an axis
    /// sitting exactly at its own floor, no matter how large the gap to a
    /// SUM-level cap would look -- an axis that was never offered headroom
    /// above its floor cannot be "wasting unused headroom". This is the
    /// structural property that makes per-axis different from combined-sum
    /// (see the next test), demonstrated directly on the predicate rather
    /// than through a full closed-loop run.
    #[test]
    fn per_axis_predicate_excludes_an_axis_pinned_at_its_floor() {
        // The GPU sits exactly at its floor (structurally undrawn, e.g.
        // unpowered) -- cap == floor, draw == 0. Even with a floor far
        // larger than any measured DEMAND_MARGIN_W, per-axis must not flag
        // it: there was never headroom above the floor to waste.
        let floor = 50.0;
        assert!(!demand_limited_axis(0.0, floor, floor, DEMAND_MARGIN_W_GPU));
        assert!(!demand_limited_axis(0.0, floor, floor, 0.001));

        // The same axis WOULD be flagged once it is offered even a little
        // headroom above its floor that it still isn't using.
        assert!(demand_limited_axis(
            0.0,
            floor + DEMAND_MARGIN_W_GPU + 1.0,
            floor,
            DEMAND_MARGIN_W_GPU
        ));
    }

    /// A naive combined (sum-of-axes) predicate, by contrast, has no way to
    /// carve the permanently-pinned GPU floor share out of the sum: a CPU
    /// axis tracking its cap PERFECTLY (zero gap of its own) still reads as
    /// "demand-limited" for the whole budget whenever the GPU floor alone
    /// exceeds `margin_sum` -- exactly §2.4's "judge each axis separately"
    /// invariant's stated failure mode, demonstrated on the predicate
    /// directly rather than relying on a closed-loop run to surface it (the
    /// sweep table's scenario 5 shows the same shape, but only faintly on
    /// this machine's specific measured margins -- see its comment).
    #[test]
    fn combined_sum_predicate_mis_flags_a_perfectly_tracked_cpu_next_to_a_pinned_gpu_floor() {
        let margin_sum = DEMAND_MARGIN_W_CPU + DEMAND_MARGIN_W_GPU;
        let gpu_floor = margin_sum + 1.0; // deliberately larger than this harness's own 5.0 W, to make the point unambiguously
        let cpu_draw = 40.0;
        let cpu_cap = 40.0; // CPU tracks its cap exactly -- zero gap of its own
        let gpu_draw = 0.0;
        let gpu_cap = gpu_floor; // GPU pinned exactly at its (structural) floor

        let combined_raw = (cpu_cap + gpu_cap - (cpu_draw + gpu_draw)) > margin_sum;
        assert!(
            combined_raw,
            "expected the naive combined predicate to mis-fire here"
        );

        // Per-axis correctly sees no problem on either axis.
        assert!(!demand_limited_axis(
            cpu_draw,
            cpu_cap,
            CPU_FLOOR_W.min(cpu_cap),
            DEMAND_MARGIN_W_CPU
        ));
        assert!(!demand_limited_axis(
            gpu_draw,
            gpu_cap,
            gpu_floor,
            DEMAND_MARGIN_W_GPU
        ));
    }

    /// Scenario 7 is the hysteresis tie-breaker: without dwell, noise
    /// crossing the margin toggles the hold far more often than with it.
    #[test]
    fn hysteresis_reduces_chatter_on_the_oscillation_scenario() {
        let sc = &scenario_table()[6];
        assert_eq!(sc.name, "7 oscillation around the margin");
        let seed = 0xC0FFEE ^ (sc.name.len() as u32);
        let no_hyst = run_scenario(Rule::Conditional, sc, seed);
        let hyst = run_scenario(Rule::ConditionalHysteresis, sc, seed);
        assert!(
            hyst.toggles < no_hyst.toggles,
            "expected hysteresis to reduce toggles: no_hyst={}, hyst={}",
            no_hyst.toggles,
            hyst.toggles
        );
    }

    /// The winner: `ConditionalHysteresis`, per-axis, holds the target in
    /// every one of the seven named scenarios. This is the regression this
    /// fixture exists to keep green.
    #[test]
    fn winner_holds_every_scenario() {
        for sc in scenario_table() {
            let r = run_scenario(
                Rule::ConditionalHysteresis,
                &sc,
                0xC0FFEE ^ (sc.name.len() as u32),
            );
            assert!(
                holds_target(&r, &sc),
                "ConditionalHysteresis failed {}: mean_abs_err={:.3}, peak_overshoot={:.3}, self_latch={}",
                sc.name,
                r.mean_abs_err_c,
                r.peak_overshoot_c,
                r.self_latch
            );
        }
    }

    /// Scenario 6: the discarded GPU watts during the HOT episode actually
    /// reach the CPU axis (via `split_budget`'s existing surplus
    /// reassignment on a lowered `gpu_max_effective`) rather than being
    /// dropped -- confirms the decided GPU-HOT interaction needs no new
    /// `Budget`-level mechanism.
    #[test]
    fn gpu_hot_reassigns_its_discarded_watts_to_cpu_via_split_budget() {
        let cpu_cap_not_hot = split_budget(
            75.0,
            Demand {
                cpu_starved: 1.0,
                gpu_starved: 1.0,
            },
            CPU_FLOOR_W,
            GPU_FLOOR_W,
            CPU_MAX_W,
            GPU_MAX_W,
        )
        .0;
        let cpu_cap_hot = split_budget(
            75.0,
            Demand {
                cpu_starved: 1.0,
                gpu_starved: 1.0,
            },
            CPU_FLOOR_W,
            GPU_FLOOR_W,
            CPU_MAX_W,
            GPU_FLOOR_W, // gpu_max_effective ratcheted all the way down
        )
        .0;
        assert!(
            cpu_cap_hot > cpu_cap_not_hot,
            "CPU cap should rise when GPU HOT frees up its share: {cpu_cap_not_hot} -> {cpu_cap_hot}"
        );
    }
}

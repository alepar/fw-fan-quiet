# Adaptation v2 (cooldown gate + 2-state Kalman) Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task.

**Goal:** Replace the Auto-mode `Trim` integrator (and disconnect the dormant RLS path) with a
command-cooldown gate plus a 2-state clamped Kalman filter over `[bias, gain]`, persisted across
sessions — per `docs/plans/2026-07-09-kalman-adaptation-design.md` (the source of truth; read it).

**Architecture:** The adaptation tier in `Controller::on_auto_sample` keeps its three-gate shape
(steadiness + achievement, now + a 30 s commanded-stationarity cooldown gate). Behind the gate,
a 2×2 Kalman filter learns a fan-RPM `bias` (RPM offset, the old trim role) and a `gain`
(multiplier on the model's GPU-slope term). The allocator inverts the *bias- and gain-corrected*
contour. State `[bias, gain]` lives in `AutoState` (covariance always fresh) and persists in the
state file next to model+LUT.

**Tech Stack:** Rust, `nalgebra` (`Matrix2`/`Vector2`), `serde`/`serde_json`, existing controller
seam (`Runner`, `Effect`, `FakeRunner`).

**Key data conventions (design §3):** learn from OBSERVED watts (20-sample window means of
`cpu_pkg_w`/`gpu_w` paired with the fan `tail_mean`), plan in COMMANDED watts. The cooldown gate
compares against the WHOLE trailing 30 s window, not the last step (staircases must be caught).

---

## Conventions for every task

- Run the full suite with `cargo test` (fast, ~2 s). Run a single test with
  `cargo test <name> -- --nocolor`. `cargo build` freely.
- `cargo test` MUST be green at every commit.
- Every commit message ends with:
  ```
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01K4RP1ADiwFXdudQWSTMk1z
  ```
- Match the codebase's dense WHY-doc-comment style (state constraints/invariants, not narration).
- Do NOT touch `/var/lib/fw-fan-quiet` and do NOT run the TUI (the app is live). Unit/controller
  tests only.

---

## Task 1: `PersistedState` gains `[bias, gain]` fields

**Files:**
- Modify: `src/state.rs:14-23` (`PersistedState` struct) and its tests.

**Step 1 — Write the failing test.** Add to `src/state.rs` `mod tests`:

```rust
#[test]
fn adapt_state_roundtrips_and_defaults_to_identity() {
    // A pre-v2 state file has no adapt fields: serde(default) must load them
    // as the identity correction (bias 0, gain 1), never panic.
    let legacy = r#"{ "model": null, "lut": null, "calibrated_at": null }"#;
    let s: PersistedState = serde_json::from_str(legacy).unwrap();
    assert_eq!(s.adapt_bias, 0.0);
    assert_eq!(s.adapt_gain, 1.0);

    // And a written pair survives the roundtrip.
    let dir = fixture_dir("adapt");
    let path = dir.join("state.json");
    let saved = PersistedState {
        adapt_bias: -137.0,
        adapt_gain: 1.15,
        ..PersistedState::default()
    };
    saved.save(&path).unwrap();
    assert_eq!(PersistedState::load(&path), saved);
    fs::remove_dir_all(&dir).unwrap();
}
```

**Step 2 — Run it, expect FAIL** (`adapt_bias`/`adapt_gain` don't exist):
`cargo test adapt_state_roundtrips -- --nocolor`

**Step 3 — Implement.** In `PersistedState` add, with the identity default so an absent field
never means "no correction is a big correction":

```rust
    /// Persisted Kalman bias (RPM offset added to the model's `c`); the
    /// covariance is deliberately NOT persisted (fresh prior each session,
    /// mirroring `ThermalModel`'s serde-skipped `P`). Defaults to the
    /// identity correction so a pre-v2 or fresh state file adapts from zero.
    #[serde(default)]
    pub adapt_bias: f64,
    /// Persisted Kalman gain (multiplier on the model's GPU-slope term).
    /// Defaults to 1.0 (identity). Reset to `[0, 1]` whenever a new
    /// calibration lands (a fresh surface invalidates old corrections).
    #[serde(default = "default_gain")]
    pub adapt_gain: f64,
```

`#[derive(Default)]` will set `adapt_gain` to `0.0`, which is wrong. Replace the derive with a
hand-written `Default` (or add a `Default` impl) so the default is the identity:

```rust
fn default_gain() -> f64 {
    1.0
}

impl Default for PersistedState {
    fn default() -> Self {
        Self {
            model: None,
            lut: None,
            calibrated_at: None,
            adapt_bias: 0.0,
            adapt_gain: 1.0,
        }
    }
}
```

Remove `Default` from the `#[derive(...)]` list on the struct (keep `Debug, Clone, PartialEq,
serde::Serialize, serde::Deserialize`). Keep `#[serde(default)]` on the struct so missing fields
fall back to `Default::default()` field-wise.

**Step 4 — Run tests, expect PASS:** `cargo test -- --nocolor` (existing state tests must stay green;
`missing_file_gives_default` etc. now assert the identity default implicitly).

**Step 5 — Commit:**
```bash
git add src/state.rs
git commit -m "feat(state): persist Kalman [bias, gain] with identity defaults"
```

---

## Task 2: `ThermalModel` bias+gain contour and corrected prediction

The allocator's contour inversion becomes `pg = (target − c − bias − a·pc) / (g·(b + e·pc))`
(design §3). Replace the `trim`-only `gpu_watts_on_contour` signature with `(target, bias, gain, pc)`
and add a corrected-prediction helper for the trust residual.

**Files:**
- Modify: `src/control/thermal_model.rs` (`gpu_watts_on_contour` ~line 244, add `predict_corrected`,
  update tests `contour_roundtrip`, `contour_clamps_negative`, `contour_none_below_half_divisor_floor`).
- Callers updated in Task 5 (controller) — the allocator's own tests build their own closures and
  are unaffected.

**Step 1 — Write failing tests.** Replace `contour_roundtrip` and extend:

```rust
#[test]
fn contour_roundtrip_with_bias_and_gain() {
    let m = exact_model();
    let target = 3000.0;
    for bias in [0.0, 100.0, -80.0] {
        for gain in [0.6, 1.0, 1.6] {
            for pc in [10.0, 25.0, 40.0] {
                let pg = m.gpu_watts_on_contour(target, bias, gain, pc).unwrap();
                assert!(pg > 0.0, "expected unclamped contour at pc={pc}");
                // The corrected model hits the target exactly at (pc, pg).
                let rpm = m.predict_corrected(pc, pg, bias, gain);
                assert!((rpm - target).abs() < 1e-6, "bias={bias} gain={gain} pc={pc} rpm={rpm}");
            }
        }
    }
}

#[test]
fn predict_corrected_is_predict_when_identity() {
    let m = exact_model();
    for (pc, pg) in [(10.0, 20.0), (30.0, 65.0)] {
        assert!((m.predict_corrected(pc, pg, 0.0, 1.0) - m.predict(pc, pg)).abs() < 1e-9);
    }
}

#[test]
fn contour_none_when_gain_shrinks_divisor_below_half_floor() {
    // Divisor is now g·(b + e·pc); the None guard keys off the SCALED divisor.
    let m = ThermalModel { a: A, b: 1.0, e: 0.0, c: C, p: None, last_rls_point: None };
    // b=1.0, gain 0.6 -> scaled divisor 0.6 < MIN_CONTOUR_DIVISOR/2 = 1.0 -> None.
    assert_eq!(m.gpu_watts_on_contour(3000.0, 0.0, 0.6, 10.0), None);
    // gain 1.6 -> 1.6 >= 1.0 -> answerable.
    assert!(m.gpu_watts_on_contour(3000.0, 0.0, 1.6, 10.0).is_some());
}
```

Update the existing `contour_clamps_negative` call sites to the new signature
(`gpu_watts_on_contour(500.0, 0.0, 1.0, 10.0)` etc.).

**Step 2 — Run, expect FAIL** (signature/`predict_corrected` missing).

**Step 3 — Implement.** Replace `gpu_watts_on_contour`:

```rust
/// The ≤target-RPM contour with the Kalman correction applied: GPU watts as a
/// function of CPU watts, `pg = (target − c − bias − a·pc) / (g·(b + e·pc))`,
/// clamped ≥ 0. `bias` shifts `c`; `g` scales the whole GPU-slope divisor.
/// `None` when the SCALED divisor `g·(b + e·pc) < MIN_CONTOUR_DIVISOR / 2`
/// (degenerate): a near-zero divisor turns the contour into thousands of
/// phantom watts (the 2026-06 field incident). With the fit-enforced
/// `b + e·pc ≥ MIN_CONTOUR_DIVISOR` and the Kalman `g ∈ [0.6, 1.6]`, a healthy
/// model's scaled divisor never drops below 1.2 — the None path only guards a
/// degenerate model inherited from disk that never passed the fit/KF clamps.
pub fn gpu_watts_on_contour(&self, target_rpm: f64, bias: f64, gain: f64, pc: f64) -> Option<f64> {
    let divisor = gain * (self.b + self.e * pc);
    if divisor < MIN_CONTOUR_DIVISOR / 2.0 {
        return None;
    }
    let pg = (target_rpm - (self.c + bias) - self.a * pc) / divisor;
    Some(pg.max(0.0))
}

/// The measurement model the Kalman filter regresses (design §3):
/// `rpm = a·pc + c + bias + g·(b·pg + e·pc·pg)`. Equals [`predict`] at the
/// identity correction `(bias=0, gain=1)`. Used for the trust residual so the
/// monitor grades the surface we are actually controlling with.
pub fn predict_corrected(&self, pc: f64, pg: f64, bias: f64, gain: f64) -> f64 {
    self.a * pc + self.c + bias + gain * (self.b * pg + self.e * pc * pg)
}
```

**Step 4 — Run tests, expect PASS.** (Only thermal_model tests should reference the old signature;
fix any leftover call in that file.)

**Step 5 — Commit:**
```bash
git add src/control/thermal_model.rs
git commit -m "feat(model): bias+gain contour inversion and corrected prediction"
```

---

## Task 3: The 2-state Kalman filter module

New module `src/control/kalman.rs`. Ports the `trim.rs` acceptance suite (design §6) plus the
gain-learning and clamp-rejection tests. The filter is pure over primitives `(measured, baseline,
w)` so it is testable without a `ThermalModel`:

- `baseline = a·pc + c` (the bias-independent part of the prediction),
- `w = b·pg + e·pc·pg` (the GPU-slope regressor the gain scales),
- prediction `ŷ = baseline + bias + gain·w`, innovation `r = measured − ŷ`.

When the allocator holds the plant on the corrected contour, `baseline + bias + gain·w = target`,
so `r = measured − target` — the same negative-feedback control error the trim was fixed to use.

**Files:**
- Create: `src/control/kalman.rs`
- Modify: `src/control/mod.rs` (add `pub mod kalman;`)

**Step 1 — Write the module with its tests** (full content):

```rust
//! 2-state clamped Kalman filter over `[bias, gain]` (design doc §3), the
//! Auto-mode adaptation tier. Supersedes the bounded `Trim` integrator (whose
//! ±400 RPM offset is now the `bias` state) and the dormant 4-param RLS: two
//! clamped degrees of freedom keep adaptation inside a box that cannot go
//! degenerate (the 2026-06 RLS divisor-drift incident is excluded by the gain
//! clamp alone).
//!
//! Measurement model, linear in the state:
//!   rpm = baseline + bias + gain·w        baseline = a·pc + c,  w = b·pg + e·pc·pg
//! regressor x = [1, w]. One textbook 2×2 KF update per gated sample with
//! process noise Q = diag(q_bias, q_gain) and measurement noise R.
//!
//! Why the split cannot double-correct (the failure that disabled 4-param
//! RLS): at a constant operating point x never changes direction, so
//! innovations move only `bias`; `gain` moves only when the operating point
//! jumps (the idle→game onset is the highest-information gain measurement,
//! weighted by accumulated covariance). No secant/anchor bookkeeping — it
//! falls out of the covariance structure.
//!
//! No positive feedback: the allocator parks the plant on the corrected
//! contour, so `baseline + bias + gain·w = target` and the innovation equals
//! the control error `measured − target` — the negative feedback the trim was
//! fixed to use (see the 2026-07 field incident in the removed `trim.rs`
//! module docs and the design doc).
//!
//! Safety contract (carried 1:1 from the trim):
//! - `bias` saturates at ±[`MAX_BIAS_AUTHORITY_RPM`] (clamped, so it can pin;
//!   `StatusFlag::TargetUnreachable` keys off the pin unchanged).
//! - `gain` clamped to [[`GAIN_MIN`], [`GAIN_MAX`]]. An update whose gain would
//!   leave that box is REJECTED WHOLE (no partial state, no covariance step —
//!   the `rls_update` convention): a poisoned/degenerate sample leaves no
//!   trace. With the fit's divisor floor of 2.0 the effective contour divisor
//!   `gain·(b + e·pc)` can never fall below 1.2.
//! - Non-finite inputs rejected outright; covariance trace capped.
//! - Cadence-gated ([`KF_PERIOD_S`]) so it can never fight the 30–90 s thermal
//!   lag — the same minutes-scale tier the trim occupied.

use nalgebra::{Matrix2, Vector2};

/// Update cadence, seconds (matches the fielded trim's 20 s tick; combined
/// with the caller's 30 s cooldown + 20-sample steadiness gates the effective
/// protection is ~50 s). The first gated call updates immediately.
pub const KF_PERIOD_S: f64 = 20.0;

/// Bias authority (RPM). Identical bound and role to the trim's
/// `MAX_TRIM_AUTHORITY_RPM`: a wrong model / blocked intake can cut the budget
/// by a bounded amount only, and the allocator's floors clamp even that.
pub const MAX_BIAS_AUTHORITY_RPM: f64 = 400.0;

/// Gain clamp. [0.6, 1.6] × the fit's `b + e·pc ≥ 2.0` divisor floor keeps the
/// effective contour divisor ≥ 1.2 — the degenerate-divisor incident class is
/// excluded by this clamp alone.
pub const GAIN_MIN: f64 = 0.6;
pub const GAIN_MAX: f64 = 1.6;

/// Measurement noise variance (RPM²). ~100 RPM of settled soak noise
/// (design §3: ±80–160 RPM on settled points).
const R: f64 = 1.0e4;
/// Bias process noise, tuned so the bias-only steady-state gain ≈ 0.05 (the
/// trim's KI): with R above, the scalar Riccati gives q_bias = k²R/(1−k) ≈ 26.
/// This is the ~7-minute-time-constant regression anchor (design §3 / §6).
const Q_BIAS: f64 = 26.0;
/// Gain process noise: far smaller — slopes change with load type, not by the
/// minute — so `gain` only moves under the covariance an operating-point jump
/// injects, never by grinding at a fixed point.
const Q_GAIN: f64 = 1.0e-4;
/// Fresh covariance prior on construction (never persisted). Bias prior sized
/// near its steady state (gentle first step like the trim); gain prior wide
/// enough (std 0.5 spans the clamp box) to learn at the first onset.
const P0_BIAS: f64 = 526.0;
const P0_GAIN: f64 = 0.25;
/// Covariance trace cap: after an accepted update, a trace above this is
/// rescaled down (belt-and-suspenders vs a drifting-point covariance blow-up,
/// as in `rls_update`).
const TRACE_CAP: f64 = 1.0e6;
const TRACE_RESCALE_TO: f64 = 1.0e5;

/// 2-state Kalman filter. Owned by the controller's Auto-mode loop state, so
/// the covariance drops (resets to the fresh prior) on Auto exit; the
/// `[bias, gain]` state is seeded from persistence on entry and written back
/// on exit (see `state.rs` / the controller).
#[derive(Debug, Clone)]
pub struct Kalman {
    theta: Vector2<f64>, // [bias, gain]
    p: Matrix2<f64>,
    last_update_t: Option<f64>,
}

impl Kalman {
    /// Seed the state from persistence (or `[0.0, 1.0]` for the identity);
    /// covariance always starts at the fresh prior.
    pub fn new(bias: f64, gain: f64) -> Self {
        Self {
            theta: Vector2::new(bias, gain),
            p: Matrix2::new(P0_BIAS, 0.0, 0.0, P0_GAIN),
            last_update_t: None,
        }
    }

    pub fn bias(&self) -> f64 {
        self.theta[0]
    }

    pub fn gain(&self) -> f64 {
        self.theta[1]
    }

    /// One cadence-gated KF update; returns true iff the state changed.
    ///
    /// `baseline = a·pc + c`, `w = b·pg + e·pc·pg` at the OBSERVED operating
    /// point (design §3: learn from observed watts). The caller owns the
    /// steadiness + achievement + cooldown gates; this owns cadence, clamps
    /// and reject-whole.
    ///
    /// - Non-finite inputs → reject (no state, no cadence-slot consumed).
    /// - The gain clamp is reject-whole: a candidate `gain` outside
    ///   [[`GAIN_MIN`], [`GAIN_MAX`]] discards the ENTIRE update (state and
    ///   covariance bit-identical) — a poisoned/degenerate sample leaves no
    ///   trace, the `rls_update` convention.
    /// - The bias clamp saturates (clamped, not rejected) so it can pin at
    ///   ±max for the `TargetUnreachable` flag, exactly like the trim.
    pub fn update(&mut self, t_mono: f64, measured: f64, baseline: f64, w: f64) -> bool {
        if let Some(last) = self.last_update_t
            && t_mono - last < KF_PERIOD_S
        {
            return false;
        }
        if !(measured.is_finite() && baseline.is_finite() && w.is_finite()) {
            return false;
        }
        self.last_update_t = Some(t_mono);

        let x = Vector2::new(1.0, w);
        // Predict (random walk): P⁻ = P + Q.
        let q = Matrix2::new(Q_BIAS, 0.0, 0.0, Q_GAIN);
        let p_pred = self.p + q;
        // Innovation against the corrected prediction.
        let y_hat = baseline + self.theta[0] + self.theta[1] * w;
        let innov = measured - y_hat;
        let s = (x.transpose() * p_pred * x)[(0, 0)] + R;
        let k = (p_pred * x) / s; // 2-vector
        let theta_new = self.theta + k * innov;

        // Gain clamp = reject-whole (degenerate/poison guard, no trace).
        if !theta_new[1].is_finite() || theta_new[1] < GAIN_MIN || theta_new[1] > GAIN_MAX {
            return false;
        }
        // Bias saturates so it can pin at ±max (flag semantics).
        let bias = theta_new[0].clamp(-MAX_BIAS_AUTHORITY_RPM, MAX_BIAS_AUTHORITY_RPM);
        let next = Vector2::new(bias, theta_new[1]);
        let changed = next != self.theta;
        self.theta = next;

        // Covariance step: P = (I − K·xᵀ)·P⁻, with the trace cap.
        let mut p_next = (Matrix2::identity() - k * x.transpose()) * p_pred;
        let trace = p_next[(0, 0)] + p_next[(1, 1)];
        if trace > TRACE_CAP {
            p_next *= TRACE_RESCALE_TO / trace;
        }
        self.p = p_next;
        changed
    }

    /// Back to the identity correction and a fresh prior. The controller
    /// resets by dropping its Auto loop state; kept for the persistence-reset
    /// path (a new calibration lands).
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        *self = Self::new(0.0, 1.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Closed-loop plant on the corrected contour: the allocator holds
    /// `baseline + bias_est + gain_est·w = target`, so `baseline = target −
    /// bias_est − gain_est·w`; the plant then reads
    /// `measured = baseline + bias_true + gain_true·w`. Mirrors the controller
    /// exactly, so the innovation is the control error `measured − target`.
    fn step_plant(kf: &Kalman, target: f64, w: f64, bias_true: f64, gain_true: f64) -> (f64, f64) {
        let baseline = target - kf.bias() - kf.gain() * w;
        let measured = baseline + bias_true + gain_true * w;
        (measured, baseline)
    }

    #[test]
    fn new_starts_at_identity() {
        let kf = Kalman::new(0.0, 1.0);
        assert_eq!(kf.bias(), 0.0);
        assert_eq!(kf.gain(), 1.0);
    }

    #[test]
    fn cadence_gates_at_20s() {
        let mut kf = Kalman::new(0.0, 1.0);
        assert!(kf.update(0.0, 2100.0, 2000.0, 0.0)); // first call integrates
        assert!(!kf.update(10.0, 2100.0, 2000.0, 0.0)); // < 20 s: gated
        assert!(!kf.update(19.9, 2100.0, 2000.0, 0.0));
        assert!(kf.update(20.0, 2100.0, 2000.0, 0.0)); // 20 s: integrates
    }

    #[test]
    fn converges_to_plant_bias_and_stops() {
        // Ported trim test: a pure bias plant (gain_true = 1, w = 0). The
        // control-error innovation drives geometric convergence to the bias.
        const BIAS: f64 = 139.0;
        const TARGET: f64 = 3250.0;
        let mut kf = Kalman::new(0.0, 1.0);
        let mut deltas = Vec::new();
        for i in 0..200 {
            let before = kf.bias();
            let (measured, baseline) = step_plant(&kf, TARGET, 0.0, BIAS, 1.0);
            kf.update(f64::from(i) * KF_PERIOD_S, measured, baseline, 0.0);
            assert!(kf.bias().abs() < MAX_BIAS_AUTHORITY_RPM, "in-authority bias must not pin");
            deltas.push((kf.bias() - before).abs());
        }
        assert!((kf.bias() - BIAS).abs() < 10.0, "bias = {}", kf.bias());
        assert!(deltas[190..].iter().all(|d| *d < 1.0), "converged bias must stop moving");
        // The plant reads back on target.
        let (measured, _) = step_plant(&kf, TARGET, 0.0, BIAS, 1.0);
        assert!((measured - TARGET).abs() < 10.0);
    }

    #[test]
    fn bias_only_time_constant_matches_trim() {
        // Regression guard on the tuning (design §6): with the trim's KI=0.05
        // at a 20 s tick, τ = 400 s ≈ 20 updates for 1−1/e ≈ 63% of a step.
        // The KF's bias-only response must land in the same ballpark.
        const BIAS: f64 = 200.0;
        const TARGET: f64 = 3000.0;
        let mut kf = Kalman::new(0.0, 1.0);
        for i in 0..20 {
            let (measured, baseline) = step_plant(&kf, TARGET, 0.0, BIAS, 1.0);
            kf.update(f64::from(i) * KF_PERIOD_S, measured, baseline, 0.0);
        }
        // ~63% of 200 after ~1 τ; allow a generous band (tuning, not exactness).
        assert!(kf.bias() > 0.45 * BIAS && kf.bias() < 0.80 * BIAS, "bias after 1τ = {}", kf.bias());
    }

    #[test]
    fn learns_gain_across_onsets() {
        // A plant whose GPU slope is 15% steeper than the fitted surface
        // (gain_true = 1.15) at a high-w operating point. Repeated onset
        // samples pull gain up toward 1.15 without pinning bias.
        const TARGET: f64 = 3000.0;
        const W: f64 = 900.0; // b·pg + e·pc·pg at a GPU-heavy point
        let mut kf = Kalman::new(0.0, 1.0);
        for i in 0..300 {
            let (measured, baseline) = step_plant(&kf, TARGET, W, 0.0, 1.15);
            kf.update(f64::from(i) * KF_PERIOD_S, measured, baseline, W);
        }
        assert!((kf.gain() - 1.15).abs() < 0.05, "gain = {}", kf.gain());
    }

    #[test]
    fn out_of_authority_bias_pins_at_max() {
        const BIAS: f64 = 550.0; // > 400 authority
        const TARGET: f64 = 3250.0;
        let mut kf = Kalman::new(0.0, 1.0);
        for i in 0..300 {
            let (measured, baseline) = step_plant(&kf, TARGET, 0.0, BIAS, 1.0);
            kf.update(f64::from(i) * KF_PERIOD_S, measured, baseline, 0.0);
        }
        assert_eq!(kf.bias(), MAX_BIAS_AUTHORITY_RPM);
    }

    #[test]
    fn gain_clamp_rejects_whole_update_bit_identical() {
        // A wild high-w innovation that would drive gain past GAIN_MAX must
        // leave the state AND covariance bit-identical (no partial step).
        let mut kf = Kalman::new(0.0, 1.0);
        let before = kf.clone_for_test();
        // Enormous positive innovation at high w -> gain would jump >> 1.6.
        assert!(!kf.update(0.0, 100_000.0, 0.0, 900.0));
        assert_eq!(kf.clone_for_test(), before);
        // Still alive: a sane sample is accepted.
        assert!(kf.update(20.0, 3050.0, 3000.0, 100.0));
    }

    #[test]
    fn non_finite_inputs_rejected_and_slot_not_consumed() {
        let mut kf = Kalman::new(0.0, 1.0);
        assert!(!kf.update(0.0, f64::NAN, 2000.0, 0.0));
        assert!(!kf.update(1.0, 2100.0, f64::INFINITY, 0.0));
        assert!(!kf.update(2.0, 2100.0, 2000.0, f64::NAN));
        assert_eq!(kf.bias(), 0.0);
        assert_eq!(kf.gain(), 1.0);
        assert!(kf.update(3.0, 2100.0, 2000.0, 0.0)); // clean first call still integrates
    }

    #[test]
    fn covariance_trace_capped_under_alternating_excitation() {
        // Two far-apart operating points keep excitation alive; the trace must
        // stay bounded after every accepted update.
        let mut kf = Kalman::new(0.0, 1.0);
        for i in 0..500 {
            let w = if i % 2 == 0 { 0.0 } else { 900.0 };
            let (measured, baseline) = step_plant(&kf, 3000.0, w, 50.0, 1.05);
            kf.update(f64::from(i) * KF_PERIOD_S, measured, baseline, w);
            assert!(kf.trace_for_test() <= TRACE_CAP, "trace blew past the cap at i={i}");
        }
    }
}
```

Add two `#[cfg(test)]` helpers on `impl Kalman` so the tests can inspect internals without making
`p`/`theta` public:

```rust
    #[cfg(test)]
    fn clone_for_test(&self) -> (Vector2<f64>, Matrix2<f64>) {
        (self.theta, self.p)
    }
    #[cfg(test)]
    fn trace_for_test(&self) -> f64 {
        self.p[(0, 0)] + self.p[(1, 1)]
    }
```

**Step 2 — Register the module.** In `src/control/mod.rs` add `pub mod kalman;` (keep alphabetical
with the existing `pub mod` lines).

**Step 3 — Run, expect FAIL first** (before adding the module) then iterate to PASS:
`cargo test kalman:: -- --nocolor`. If the tuning tests (`bias_only_time_constant_matches_trim`,
`learns_gain_across_onsets`) miss their bands, adjust `Q_BIAS`/`Q_GAIN`/`P0_*`/`R` — these constants
are the knobs the design §3 tuning anchors describe. Do NOT loosen the assertions to hide a
mis-tuned filter; the bands are generous on purpose.

**Step 4 — Full suite green:** `cargo test`.

**Step 5 — Commit:**
```bash
git add src/control/kalman.rs src/control/mod.rs
git commit -m "feat(kalman): 2-state clamped [bias, gain] filter with ported trim suite"
```

---

## Task 4: Command-cooldown gate

A pure function + a small ring in `AutoState`. The gate opens only when the commanded operating
point has been stationary (within 2 W on both legs) for the WHOLE trailing 30 s — comparing against
the entire window, not the last step, so the +2 W/5 s staircase that caused the incident stays
blocked (design §0).

**Files:**
- Create: `src/control/cooldown.rs`
- Modify: `src/control/mod.rs` (`pub mod cooldown;`)

**Step 1 — Write the module + tests** (full content):

```rust
//! Adaptation command-cooldown gate (design doc §0, the primary fix for the
//! 2026-07-09 wind-up incident). The adaptation tier may consume a sample only
//! if the COMMANDED operating point `(cpu_w allocation, gpu_w PI target)` has
//! stayed within [`TOL_W`] on both legs for the entire trailing [`WINDOW_S`].
//!
//! Compares against the WHOLE window, not the last step: a per-step threshold
//! is evaded by exactly the +2 W/5 s staircase that walked the trim to the
//! −400 pin (the same lesson as the RLS excitation gate's drifting-point
//! caveat). Gates on commanded values only — the observed side is owned by the
//! achievement gate (drawn ≈ commanded) and `is_steady` (fan end); each gate
//! owns one edge of `command → drawn power → fan RPM`.

/// Stationarity tolerance per leg (W). 2 W clears steady-state allocator/PI
/// dither (observed ±1.9 W holds) without letting a staircase through.
pub const TOL_W: f64 = 2.0;
/// Trailing window (s). 30 s; the 20-sample `is_steady` gate stacks on top for
/// ~50 s of combined protection, and the KF noise model absorbs the residual
/// heat-soak error.
pub const WINDOW_S: f64 = 30.0;

/// One commanded operating point in the ring.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CommandedPoint {
    pub t_mono: f64,
    pub cpu_w: f64,
    pub gpu_w: f64,
}

/// True iff the ring proves the commanded point held within [`TOL_W`] on both
/// legs across the full trailing [`WINDOW_S`] ending at `now`:
/// - COVERAGE: at least one recorded point is `≥ WINDOW_S` old (otherwise we
///   have not yet observed a full window of stationarity — gate stays closed,
///   e.g. the first 30 s after Auto entry or after any move).
/// - STATIONARITY: every point within the trailing window is within `TOL_W` of
///   the current (latest) commanded point on both legs.
pub fn cooldown_open(ring: &[CommandedPoint], now: f64) -> bool {
    let Some(cur) = ring.last() else {
        return false;
    };
    let has_coverage = ring.iter().any(|p| now - p.t_mono >= WINDOW_S);
    if !has_coverage {
        return false;
    }
    ring.iter()
        .filter(|p| now - p.t_mono <= WINDOW_S)
        .all(|p| (p.cpu_w - cur.cpu_w).abs() <= TOL_W && (p.gpu_w - cur.gpu_w).abs() <= TOL_W)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(t: f64, cpu: f64, gpu: f64) -> CommandedPoint {
        CommandedPoint { t_mono: t, cpu_w: cpu, gpu_w: gpu }
    }

    #[test]
    fn empty_or_short_history_is_closed() {
        assert!(!cooldown_open(&[], 100.0));
        // 20 s of perfectly flat history: no full-window coverage yet.
        let ring: Vec<_> = (0..=20).map(|i| pt(80.0 + f64::from(i), 30.0, 60.0)).collect();
        assert!(!cooldown_open(&ring, 100.0));
    }

    #[test]
    fn flat_history_reopens_after_30s() {
        // Flat for exactly 30 s -> open the moment coverage is reached.
        let ring: Vec<_> = (0..=30).map(|i| pt(70.0 + f64::from(i), 30.0, 60.0)).collect();
        assert!(cooldown_open(&ring, 100.0));
    }

    #[test]
    fn staircase_never_opens() {
        // The incident shape: GPU walks +2 W every 5 s. Even though each
        // ADJACENT step is only 2 W, the whole-window compare sees 12 W of
        // spread across 30 s -> closed.
        let ring: Vec<_> = (0..=30)
            .map(|i| pt(70.0 + f64::from(i), 30.0, 40.0 + 0.4 * f64::from(i)))
            .collect();
        assert!(!cooldown_open(&ring, 100.0));
    }

    #[test]
    fn small_dither_does_not_close_the_gate() {
        // ±1.9 W steady-state dither around a hold: within TOL_W -> open.
        let ring: Vec<_> = (0..=30)
            .map(|i| {
                let d = if i % 2 == 0 { 1.9 } else { -1.9 };
                pt(70.0 + f64::from(i), 30.0 + d, 60.0 - d)
            })
            .collect();
        assert!(cooldown_open(&ring, 100.0));
    }

    #[test]
    fn a_move_closes_then_reopens_30s_later() {
        // Flat, then a 10 W GPU jump at t=100; sampled 1 Hz afterward.
        let mut ring: Vec<_> = (60..=100).map(|i| pt(f64::from(i), 30.0, 60.0)).collect();
        // Jump to 70 W at t=100 and hold.
        for i in 100..=125 {
            ring.push(pt(f64::from(i), 30.0, 70.0));
        }
        // 25 s after the move: the old 60 W points are still in the 30 s
        // window -> 10 W spread -> closed.
        assert!(!cooldown_open(&ring, 125.0));
        // 30 s after the move: only 70 W points remain in-window -> open.
        for i in 126..=130 {
            ring.push(pt(f64::from(i), 30.0, 70.0));
        }
        assert!(cooldown_open(&ring, 130.0));
    }
}
```

**Step 2 — Register + run tests, expect PASS** after adding `pub mod cooldown;`:
`cargo test cooldown:: -- --nocolor`.

**Step 3 — Full suite green:** `cargo test`.

**Step 4 — Commit:**
```bash
git add src/control/cooldown.rs src/control/mod.rs
git commit -m "feat(cooldown): 30 s whole-window commanded-stationarity gate"
```

---

## Task 5: Rewire the controller adaptation tier onto the Kalman filter

This is the central change. Replace `AutoState.trim: Trim` with `kf: Kalman`, add the observed-watts
windows and the commanded ring, gate the tier on cooldown + steadiness + achievement, run one KF
update from OBSERVED-watts window means, feed the trust monitor the KF-corrected residual, mirror
`bias`/`gain` into status, and drop the RLS call site. Do it in small commits.

**Files:**
- Modify: `src/control/controller.rs` (imports; `ControlStatus`; `AutoState`; `on_auto_sample`
  ~lines 912-1235; `release_to_stock`; constants; the `MAX_TRIM_AUTHORITY_RPM`/`Trim` usages).

### 5a — Status carries `gain`; imports switch to `kalman`

**Step 1.** In `ControlStatus` (near `trim_rpm`, line ~237) add:

```rust
    /// Current Kalman gain (multiplier on the model's GPU-slope term); 1.0
    /// outside Auto and until the filter moves it. Shown dim in the header
    /// next to the trim/bias readout.
    pub gain: f64,
```

Set `gain: 1.0` in `impl Default for ControlStatus` (line ~259, next to `trim_rpm: 0.0`).
`trim_rpm` KEEPS its name and meaning — it is now the KF `bias` (still an RPM offset on `c`, still
±400, still what `TargetUnreachable` keys off). Update its doc-comment to say "Kalman bias".

**Step 2.** Change the imports (line ~31-32):
```rust
use crate::control::cooldown::{self, CommandedPoint};
use crate::control::kalman::{Kalman, MAX_BIAS_AUTHORITY_RPM};
```
Remove `use crate::control::trim::{MAX_TRIM_AUTHORITY_RPM, Trim};`. Replace every
`MAX_TRIM_AUTHORITY_RPM` in this file with `MAX_BIAS_AUTHORITY_RPM` (the flag threshold + hysteresis
at lines ~1215-1219, and any test references).

**Step 3.** `cargo build` — expect errors in `AutoState`/`on_auto_sample`; fix in 5b/5c. Do not
commit yet (tree not green). If you want an intermediate green commit, do 5a+5b+5c together.

### 5b — `AutoState`: Kalman + observed windows + commanded ring

**Step 1.** In `AutoState` (line ~317) replace `trim: Trim` with:

```rust
    /// 2-state Kalman filter (design §1): `[bias, gain]`. Seeded from
    /// persistence on Auto entry, covariance fresh; written back on Auto exit.
    kf: Kalman,
    /// 20-sample OBSERVED-watts windows (design §3: learn from observed watts).
    /// Invalid samples land as NaN so `tail_mean` refuses to average across a
    /// sensor outage — paired with `fan_window`'s tail into one KF update.
    cpu_w_window: std::collections::VecDeque<f64>,
    gpu_w_window: std::collections::VecDeque<f64>,
    /// Ring of recent COMMANDED points for the cooldown gate (design §0). One
    /// entry per sample; capped past the 30 s window.
    commanded_ring: std::collections::VecDeque<CommandedPoint>,
```

Keep `fan_window`, `trust`, `distrusted`, `last_snapshot`, `pid`, `allocator`, `last_alloc`,
`gpu_target_w`.

**Step 2.** `AutoState::new` becomes seeded:

```rust
    fn new(bias: f64, gain: f64) -> Self {
        Self {
            pid: GpuPid::new(),
            allocator: Allocator::new(),
            last_alloc: None,
            gpu_target_w: None,
            kf: Kalman::new(bias, gain),
            fan_window: std::collections::VecDeque::new(),
            cpu_w_window: std::collections::VecDeque::new(),
            gpu_w_window: std::collections::VecDeque::new(),
            commanded_ring: std::collections::VecDeque::new(),
            trust: TrustMonitor::new(),
            distrusted: false,
            last_snapshot: None,
        }
    }
```

Update the `AutoState::new()` call site in `Command::SetAuto(true)` (line ~658) to
`AutoState::new(self.persisted_bias, self.persisted_gain)` — those controller fields are added in
Task 6; for THIS task use `AutoState::new(0.0, 1.0)` and switch it in Task 6.

Add a ring cap constant near the other Auto constants:
```rust
/// Cap on the cooldown ring (~35 s of 1 Hz samples ≥ the 30 s gate window).
const COMMANDED_RING_CAP: usize = 40;
```

**Step 3.** `cargo build` — errors now localize to `on_auto_sample`; fix in 5c.

### 5c — `on_auto_sample`: windows, contour, gate, KF, trust

Work through `on_auto_sample` (lines ~912-1235):

**Step 1 — observed windows.** Right after the fan-window push (line ~937-944), push the observed
watts (same NaN-on-invalid convention). The fan-invalid check already exists; add:

```rust
        // Observed-watts windows (design §3): NaN on invalid so `tail_mean`
        // refuses to average across an outage.
        for (win, valid, val) in [
            (&mut auto.cpu_w_window, s.cpu_pkg_w > 0.0, s.cpu_pkg_w),
            (&mut auto.gpu_w_window, s.gpu_w_valid, s.gpu_w),
        ] {
            if win.len() >= FAN_WINDOW_CAP {
                win.pop_front();
            }
            win.push_back(if valid { val } else { f64::NAN });
        }
```

**Step 2 — contour uses bias + gain.** In the allocator step (line ~964-970) replace:
```rust
            let trim_rpm = auto.trim.offset_rpm();
            ...
            let contour = |pc: f64| model.gpu_watts_on_contour(target_rpm, trim_rpm, pc);
```
with:
```rust
            let bias = auto.kf.bias();
            let gain = auto.kf.gain();
            ...
            let contour = |pc: f64| model.gpu_watts_on_contour(target_rpm, bias, gain, pc);
```

**Step 3 — record the commanded point every sample.** After the allocator block (after line ~1017,
before the GPU PI), push the current commanded operating point onto the ring — using the freshest
`cpu_limit_w`/`gpu_target_w`. Only record once both legs exist (early samples before the first
alloc have no commanded point):

```rust
        // Cooldown ring (design §0): the commanded operating point this sample.
        if let (Some(pc), Some(pg)) = (self.status.cpu_limit_w, auto.gpu_target_w) {
            if auto.commanded_ring.len() >= COMMANDED_RING_CAP {
                auto.commanded_ring.pop_front();
            }
            auto.commanded_ring.push_back(CommandedPoint { t_mono: s.t_mono, cpu_w: pc, gpu_w: pg });
        }
```
(`self.status.cpu_limit_w` reborrow: `auto` is `self.auto.as_mut()`. To avoid a borrow conflict,
read `self.status.cpu_limit_w` into a local before the `auto` borrow, or restructure — the existing
code already reads `self.status.*` while holding `auto`, so follow the established pattern. If the
borrow checker complains, copy `let cpu_cmd = self.status.cpu_limit_w;` up top.)

**Step 4 — replace the adaptation block.** Replace the whole `if s.fan_valid && is_steady && ...
achievement { trust/RLS/trim }` block (lines ~1118-1187) with the cooldown-gated KF. Keep the
achievement-gate doc-comment block above it (it still applies verbatim — trim it to say "the KF"
instead of "the trim"). New block:

```rust
        // Adaptation tier (design v2): four gates in series, each owning one
        // edge of `command → drawn power → fan RPM`:
        //   1. cooldown  — the COMMANDED point held ±2 W for the whole trailing
        //      30 s (design §0; catches the +2 W/5 s staircase the last-step
        //      test would miss — the 2026-07 wind-up).
        //   2. is_steady — the fan window (20 samples) settled.
        //   3. achievement — the load actually DREW the commanded budget.
        //   4. finite observed-watts window means (never learn across an outage).
        // Behind them: trust verdict, then (if trusted) one KF update. The KF
        // learns from OBSERVED-watts window means and plans in commanded watts.
        let now = s.t_mono;
        let cur_cmd = (self.status.cpu_limit_w, auto.gpu_target_w);
        if s.fan_valid
            && cooldown::cooldown_open(auto.commanded_ring.make_contiguous(), now)
            && is_steady(auto.fan_window.make_contiguous(), STEADY_N, STEADY_RPM_TOLERANCE)
            && let (Some(cpu_cmd), Some(gpu_cmd)) = cur_cmd
            && s.cpu_pkg_w >= cpu_cmd - ACHIEVED_CPU_MARGIN_W
            && s.gpu_w >= gpu_cmd - ACHIEVED_GPU_MARGIN_W
            && let Some(measured) = tail_mean(auto.fan_window.make_contiguous(), STEADY_N)
            && let Some(pc_obs) = tail_mean(auto.cpu_w_window.make_contiguous(), STEADY_N)
            && let Some(pg_obs) = tail_mean(auto.gpu_w_window.make_contiguous(), STEADY_N)
        {
            let model = self.model.as_ref().expect("checked above");
            // KF primitives at the OBSERVED operating point.
            let baseline = model.a * pc_obs + model.c;
            let w = model.b * pg_obs + model.e * pc_obs * pg_obs;
            let corrected = baseline + auto.kf.bias() + auto.kf.gain() * w;
            let residual = (measured - corrected).abs();
            // Trust verdict first (it decides whether this sample may adapt).
            auto.distrusted = auto.trust.observe(now, residual) == Trust::Distrust;
            // KF frozen entirely while distrusted (suspect evidence is rare and
            // discrete; not worth half-weighting — design §1).
            if !auto.distrusted && auto.kf.update(now, measured, baseline, w) {
                self.status.trim_rpm = auto.kf.bias();
                self.status.gain = auto.kf.gain();
                cause.get_or_insert("auto:kf");
            }
        }
```

Remove the RLS wiring: delete the `Effect::RlsAccepted` push (the effect enum variant and its
shell/telemetry handling can stay for now to minimize churn, but nothing emits it — OR remove it in
5d). Remove the `RLS_LAMBDA` constant and the `self.config.online_rls` read. Remove `DISTRUST_TRIM_KI_SCALE`
usage (the KF has no half-gain; it freezes). `use ...steady::{... tail_mean}` is already imported.

**Step 5 — snapshot/flag tails.** Further down (lines ~1188-1234) replace
`let offset = auto.trim.offset_rpm();` with `let offset = auto.kf.bias();` (the `TargetUnreachable`
block keys off `offset` unchanged, now against `MAX_BIAS_AUTHORITY_RPM`). The `ModelSnapshot` block
stays (still carries a/b/e/c). Leave `distrusted`/ModelDistrust mirroring as-is.

**Step 6 — `release_to_stock`.** After `self.status.trim_rpm = 0.0;` (line ~1456) add
`self.status.gain = 1.0;` so an Auto exit resets the visible gain too.

**Step 7 — fix the controller's own tests.** Several existing tests reference the old behavior:
- Tests asserting `Effect::RlsAccepted` / `online_rls` / `auto:rls` telemetry / `auto:trim` cause:
  update the cause string to `"auto:kf"`, or delete the RLS-specific tests
  (`online_rls_never_touches_the_persisted_state`, the `auto:rls` telemetry assertions) since the
  path is gone. Keep the SPIRIT: adaptation still never writes the persisted CALIBRATED params
  during a session — that is now covered by Task 6's persistence tests, so deleting the RLS variant
  is fine.
- Tests driving the trim (e.g. `persistent_residual_at_frozen_point_saturates_trim_and_flags_unreachable`,
  the `trim_rpm` convergence tests): these now exercise the KF via the controller. They should still
  pass IF they feed a steady, achieved, cooldown-OPEN sequence. The critical new requirement: to
  open the cooldown gate the test must hold the commanded point flat for ≥30 s. Many trim tests hold
  a fixed operating point already; they just need enough samples. Where a test fails only because
  the gate stays closed, extend the flat-hold duration; do NOT weaken the gate. Where the cause
  string changed (`auto:trim` → `auto:kf`), update the assertion.
- The `header_shows_trim_*` and telemetry `trim_rpm` tests keep working (name unchanged).

Work test-by-test: run `cargo test --lib control::controller 2>&1 | tail -40`, fix the first
failure, repeat. This is the bulk of the task.

**Step 8 — Commit** (5a+5b+5c together, once green):
```bash
git add src/control/controller.rs
git commit -m "feat(controller): adaptation tier on the cooldown-gated Kalman filter"
```

### 5d — (optional cleanup) retire the `Effect::RlsAccepted` path

If the `RlsAccepted` variant and its shell handling are now dead, remove `Effect::RlsAccepted`, the
`rls_accepted` handling in `apply_effects`, the `auto:rls` decision line, and any remaining
`online_rls`/`RLS_LAMBDA` references. Keep `ThermalModel::rls_update` and its unit tests in
`thermal_model.rs` intact (they still validate the divisor-floor math the contour relies on; the
method is `pub`, so it will not warn as dead). Run `cargo test`, commit:
```bash
git commit -am "refactor(controller): remove the retired online-RLS effect path"
```

---

## Task 6: Cross-session persistence of `[bias, gain]`

Seed the KF from the state file on Auto entry; write `[bias, gain]` on every Auto exit and on clean
quit; reset to `[0, 1]` when a new calibration lands (design §2). Never persist covariance.

**Files:**
- Modify: `src/control/controller.rs` (`Controller` fields + `new`; Auto entry/exit sites; Quit;
  the `RunnerEffect::SaveState` calibration path; a `save_persisted_state` helper).

**Step 1 — Controller fields.** Add to `Controller` (near `model`/`lut`, line ~401):

```rust
    /// Wall-clock timestamp of the loaded calibration, carried so an Auto-exit
    /// state write preserves it (only a calibration sets it).
    calibrated_at: Option<String>,
    /// Persisted Kalman seed for the NEXT Auto entry (design §2). Loaded from
    /// the state file; captured from the live filter on every Auto exit;
    /// reset to the identity `[0, 1]` when a new calibration lands.
    persisted_bias: f64,
    persisted_gain: f64,
```

In `Controller::new` (line ~457-475) initialize them from `persisted`:
```rust
            calibrated_at: persisted.calibrated_at,
            persisted_bias: persisted.adapt_bias,
            persisted_gain: persisted.adapt_gain,
```
(Note: `persisted` is consumed field-by-field already for `model`/`lut`; read `calibrated_at`/
`adapt_*` before or alongside those moves — reorder so `persisted.model`/`persisted.lut` moves come
after the copies, or copy the `f64`s first.)

**Step 2 — `save_persisted_state` helper.** Add a method:

```rust
    /// Write model + LUT + Kalman `[bias, gain]` to the state file (design §2:
    /// covariance is never persisted). Called on Auto exit and clean quit —
    /// NOT per update (no disk churn). Save failure is warned, not fatal: the
    /// in-memory state stands.
    fn save_persisted_state(&self) {
        let state = PersistedState {
            model: self.model.clone(),
            lut: self.lut.clone(),
            calibrated_at: self.calibrated_at.clone(),
            adapt_bias: self.persisted_bias,
            adapt_gain: self.persisted_gain,
        };
        if let Err(e) = state.save(&self.state_path) {
            tracing::warn!("adapt: state save to {} failed: {e}", self.state_path.display());
        }
    }
```

**Step 3 — capture-on-exit helper.** Add:

```rust
    /// Drop the Auto loop state, first capturing the live Kalman `[bias, gain]`
    /// into the persisted seed and writing the state file (design §2: persist
    /// on Auto exit). No-op beyond the state write if not currently in Auto.
    fn exit_auto_and_persist(&mut self) {
        if let Some(auto) = self.auto.take() {
            self.persisted_bias = auto.kf.bias();
            self.persisted_gain = auto.kf.gain();
        }
        self.save_persisted_state();
    }
```

**Step 4 — wire the exit sites.** Replace `self.auto = None;` with `self.exit_auto_and_persist();`
at the Auto-exit paths:
- `Command::ReleaseAll` (line ~593),
- `Command::SetAuto(false)` (line ~682),
- `emergency_release` (line ~1424).
For `Command::Quit` (line ~716-728): before `self.guard.restore_all()`, call
`self.exit_auto_and_persist();` (captures the live filter if quitting from Auto, and writes state
either way — the "clean quit" requirement).

Leave the `on_auto_sample` degraded-path `self.auto = None;` (line ~924) as a plain drop (it is a
fault release toward stock; persisting a half-broken session is undesirable, and the seed from the
last clean exit stands). Document that choice with a one-line comment.

**Step 5 — seed on entry.** In `Command::SetAuto(true)` change `AutoState::new(0.0, 1.0)` (from Task
5) to `AutoState::new(self.persisted_bias, self.persisted_gain)`.

**Step 6 — reset on calibration.** In `apply_calib_effects`, `RunnerEffect::SaveState` (line ~1358):
after setting `self.model`/`self.lut`, also reset and record the seed + timestamp:
```rust
                RunnerEffect::SaveState(state) => {
                    self.model = state.model.clone();
                    self.lut = state.lut.clone();
                    self.calibrated_at = state.calibrated_at.clone();
                    // A fresh surface invalidates old corrections (design §2).
                    self.persisted_bias = 0.0;
                    self.persisted_gain = 1.0;
                    ...
```
(The `state.save(&self.state_path)` there already writes a `PersistedState` whose `adapt_*` are the
identity defaults, so the on-disk reset is automatic; this just keeps the in-memory seed in sync.)

**Step 7 — tests.** Add controller tests (use the existing `FakeRunner`/temp-state-path harness —
copy the setup from `full_calibration_persists_state_and_keeps_model` / `persisted_state_seeds_controller_model_and_lut`):

```rust
#[test]
fn auto_exit_persists_kalman_state_to_disk() {
    // Enter Auto, drive a steady+achieved+cooldown-open sequence until the KF
    // moves bias off zero, then SetAuto(false); the state file must carry the
    // learned [bias, gain], and a fresh controller loaded from it must seed
    // the next Auto session with them (bias visible immediately in status).
}

#[test]
fn seeded_bias_is_live_from_the_first_auto_sample() {
    // Load a PersistedState with adapt_bias = -120, adapt_gain = 1.1; entering
    // Auto, the first allocator step must invert the SEEDED contour (assert
    // status.trim_rpm == -120 / status.gain == 1.1 right after entry, before
    // any adaptation).
}

#[test]
fn new_calibration_resets_persisted_adaptation() {
    // Persist a non-identity [bias, gain]; run a full calibration; the saved
    // state's adapt_bias/adapt_gain must be back to [0, 1].
}
```

Implement the driving loop by reusing the pattern in the existing trim convergence tests (feed
`Sample`s at 1 Hz with a flat commanded point long enough to open the 30 s cooldown gate, fan window
steady, `cpu_pkg_w`/`gpu_w` at/above the commanded budget). For `seeded_bias_is_live`, the assertion
is immediate (no adaptation needed) — just enter Auto and push one sample so the allocator runs.

**Step 8 — Commit:**
```bash
git add src/control/controller.rs
git commit -m "feat(controller): persist and seed Kalman [bias, gain] across sessions"
```

---

## Task 7: Telemetry `gain` field + UI header gain readout

Make both `bias` (already `trim_rpm`) and `gain` visible offline and in the header (design §3
consumers; mission constraint).

**Files:**
- Modify: `src/telemetry.rs` (`Record::Decision` + the ordered-keys test ~line 373).
- Modify: `src/control/controller.rs` (`decision` closure in `apply_effects` ~line 1670-1700).
- Modify: `src/ui/view.rs` (`header_line` ~line 99-106) + a header test.

**Step 1 — telemetry field.** In `Record::Decision` add after `trim_rpm`:
```rust
        /// Current Kalman gain (multiplier on the model's GPU-slope term);
        /// carried on every Auto-mode decision alongside `trim_rpm` (the bias),
        /// None otherwise.
        #[serde(skip_serializing_if = "Option::is_none")]
        gain: Option<f64>,
```
Update the ordered-keys test (line ~373) to include `"gain"` right after `"trim_rpm"`, and set
`gain: None` in the telemetry module's own test record (line ~316).

**Step 2 — populate it.** In `apply_effects`'s `decision` closure (line ~1694) add:
```rust
            gain: (status.mode == Mode::Auto).then_some(status.gain),
```

**Step 3 — telemetry test.** Extend an Auto-decision telemetry test (near the `trim_rpm` telemetry
assertions ~line 3650) to assert the `gain` key is present on Auto lines and absent off-Auto — mirror
the existing `trim_rpm` assertions.

**Step 4 — UI header.** After the `trim` span block (line ~106) add a dim gain readout when the gain
is not the identity:
```rust
    // Kalman gain (Auto mode): dim, like the trim/bias readout. Shown only
    // when it has moved off the identity (a learned GPU-slope correction).
    if (model.status.gain - 1.0).abs() > 1e-6 {
        spans.push(Span::styled(
            format!(" | gain x{:.2}", model.status.gain),
            Style::default().fg(Color::DarkGray),
        ));
    }
```

**Step 5 — UI test.** Add to `src/ui/view.rs` tests (copy a `header_shows_trim_dim_when_nonzero`
neighbor, giving `ControlStatus { gain: 1.12, .. }`):
```rust
#[test]
fn header_shows_gain_dim_when_off_identity() {
    // status with gain 1.12 -> "gain x1.12" rendered dim; gain 1.0 -> hidden.
}
```
Every `ControlStatus { .. }` literal in the view tests needs the new `gain` field — add `gain: 1.0`
to each (the compiler lists them).

**Step 6 — Run tests, expect PASS:** `cargo test`.

**Step 7 — Commit:**
```bash
git add src/telemetry.rs src/control/controller.rs src/ui/view.rs
git commit -m "feat(telemetry,ui): surface Kalman gain alongside bias"
```

---

## Task 8: Controller-level replay of the 2026-07-09 wind-up incident

The acceptance test the design §6 calls out: feed the captured onset shape (staircase allocation +
lagging first-order fan response) through the real `Controller`+`FakeRunner` and assert the KF
consumes ZERO updates during wind-up, `bias` stays ≈ 0, and no `ModelDistrust` fires.

**Files:**
- Modify: `src/control/controller.rs` tests (new test `wind_up_staircase_produces_no_adaptation`).

**Step 1 — Write the test.** Model the incident: idle → GPU load onset; the allocator naturally
walks GPU budget up ~+2 W/5 s while the fan plant lags with a first-order response (τ ≈ 15 s, dead
time ~10 s), so the fan window passes `is_steady` (slow coordinated ramp) but the COMMANDED point
moves every 5 s. Drive the controller from Auto entry through ~3 minutes of onset. Assertions:

```rust
#[test]
fn wind_up_staircase_produces_no_adaptation() {
    // Reproduces the 2026-07-09 incident at the controller level. During the
    // staircase wind-up the commanded operating point moves every 5 s, so the
    // cooldown gate (design §0) stays shut for the WHOLE window even though the
    // fan trace is "steady" (a slow coordinated ramp) and the budget is drawn.
    // The KF must therefore never update: bias stays ~0 and no ModelDistrust.
    // ... set up Controller with a calibrated model whose surface UNDER-predicts
    //     RPM (the field bias), enter Auto, then for ~200 s feed 1 Hz samples
    //     where cpu_pkg_w/gpu_w track the commanded allocation and the fan RPM
    //     follows a lagging first-order response to the commanded GPU watts ...
    assert_eq!(ctl.status().trim_rpm, 0.0, "bias must not move during wind-up");
    assert!(!ctl.status().flags.contains(&StatusFlag::ModelDistrust));
}
```

Reuse the field-plant math already proven in `allocator.rs`'s `simulate_field_cycle`
(`SIM_PLANT_K`/`C`/`TAU_S`/`DEAD_S`, first-order + dead-time) as the fan response; drive the
controller's real samples so its OWN allocator produces the staircase. The key invariant to assert
is the ZERO-update / bias≈0 outcome; a helpful secondary assertion is that the cooldown gate is what
blocks it (e.g. after the onset ENDS and the point holds flat for ≥30 s, the KF finally updates —
optional extension).

**Step 2 — Run, expect PASS** (if bias moves, the gate or its wiring is wrong — debug the gate, do
not weaken the assertion). `cargo test wind_up_staircase -- --nocolor`.

**Step 3 — Full suite green:** `cargo test`.

**Step 4 — Commit:**
```bash
git add src/control/controller.rs
git commit -m "test(controller): 2026-07-09 wind-up replay asserts zero adaptation"
```

---

## Task 9: Final sweep

**Step 1.** `cargo test` — all green. `cargo build --release` to confirm no warnings-as-errors issues.
Run `cargo clippy` if the repo uses it (check for a CI config); fix any new lints in the touched files.

**Step 2.** Re-read design doc §6: confirm every listed test exists (KF suite, cooldown staircase/dither/
reopen, controller replay, persistence roundtrip + seed + reset, bias-only τ regression guard).

**Step 3.** Update `docs/plans/2026-07-09-kalman-adaptation-design.md`? No — the design is the frozen
source of truth. Do NOT edit it.

**Step 4 — Commit any stragglers** and produce the final report: commit hashes + one-liners, test
results, how the incident-replay test behaves, deviations (e.g. `trim.rs` retained as the allocator
velocity-gate test's plant model; `ThermalModel::rls_update` kept as tested-but-unused), and the
field-validation checklist from design §6 for the user's next session.

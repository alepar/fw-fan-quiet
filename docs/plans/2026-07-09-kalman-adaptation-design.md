# Online adaptation v2: cooldown gate + 2-state Kalman filter — Design

Replaces the Auto-mode adaptation tier (offset `Trim` + the dormant 4-param RLS) with
a command-cooldown gate and a 2-state clamped Kalman filter over `[bias, gain]`, with
cross-session persistence. Design validated against a live-captured field incident
(2026-07-09, telemetry `run-1783646589.jsonl`); the analysis below cites it throughout.

## 1. Problem: the oscillation, mechanically

Observed for months as "oscillating behavior around the fan target until controlled
variables settle" after a load onset (idle → GPU-intensive game). Captured end to end
on 2026-07-09 (onset at `t_mono≈6759`):

1. **Wind-up.** The allocator walks GPU budget up in a +2 W / 5 s staircase. Fans lag
   30+ s behind, so the whole convoy moves slower than ~5 RPM/s — every 20-sample fan
   window passes `is_steady` (spread ≤ 100 RPM), and the achievement gate passes too
   (each step is genuinely consumed). The trim integrates the large below-target
   control error at every 20 s tick: −71 → −143 → −208 → −273 → −335 → −400 (pinned
   at −max) within 3 minutes of onset.
2. **Overshoot.** The −400 trim shifts the contour up ~400 RPM worth of watts: GPU
   allocation raced 26 → 86 W and the fans peaked at 3561 RPM against a 2750 target
   (+810 RPM overshoot).
3. **Limit cycle.** The overshoot backstop cuts power, fans fall toward target, the
   allocator re-raises toward the still-poisoned contour, fans overshoot again. Trim
   updates keep sampling the loop's own transients (it went −349 → −355 *during* the
   second cycle), so the trim hovers near −350 instead of unwinding — a
   self-sustaining oscillation.
4. **False distrust.** The trust monitor, fed the same mid-cycle residuals, fired
   `ModelDistrust` at t+452, halving the trim gain — entrenching the fault it exists
   to catch.
5. **Session poisoning.** When the load ended, the achievement gate froze adaptation
   with the trim parked at −342. Idle rarely achieves the commanded budget, so the
   next onset would start ~340 RPM over-allocated from the first second.

Root cause: **fan-trace flatness cannot distinguish equilibrium from a slow
coordinated ramp.** Quantified from the same session's 135 trim updates: the gate was
never violated on its own terms (0 windows with spread > 100 RPM), yet with commanded
allocation AND measured power held static afterwards, ~20–25 % of "steady" points
drifted > 100 RPM over the following 1–3 minutes (p50 ≈ 80, p90 ≈ 160, max 222 RPM) —
multi-minute chassis heat soak, invisible to any trailing-window fan check. A
10 s fan-slope rail was tested against this data and discriminates nothing (blocked
and passed groups drift identically); it is rejected.

Secondary problem (the original complaint): the calibrated GPU-watt slope is
shallower than reality for game loads — calibration used a more aggressive burn load,
and (side finding) calibration points recorded at first-steadiness sit ~50–160 RPM
below true equilibrium, worse at high power, which systematically flattens the fitted
slope. A wrong slope makes the model's bias vary with operating point, so each load
step used to hand the trim a fresh bias to grind out at its ~7-minute time constant.

## 2. Design §0 — Adaptation cooldown gate (primary fix)

The adaptation tier (KF, trust monitor; RLS if ever re-enabled) may consume a sample
only if the **commanded operating point has been stationary for the trailing 30 s**:
every commanded `(cpu_w allocation, gpu_w PI target)` in the last 30 s within 2 W of
the current one, on both legs.

- Implementation: a small ring of the last ~30 commanded points `(t_mono, pc, pg)` in
  `AutoState`; the gate compares against the whole trailing window, **not** the last
  step — a per-step threshold is evaded by exactly the +2 W/5 s staircase that caused
  the incident (same lesson as the RLS excitation gate's drifting-point caveat).
- The 2 W tolerance keeps steady-state allocator/PI dither (observed ±1.9 W holds)
  from blocking adaptation forever.
- 30 s (not more) because the existing 20-sample `is_steady` still applies on top:
  worst case ~50 s of combined protection, and the KF noise model (§3) absorbs the
  residual heat-soak error.
- Gates on **commanded values only**. The observed side is owned by the two existing
  gates: achievement (drawn ≈ commanded within 3/5 W margins — the "did the command
  get acted on" check) and `is_steady` (fan end). Each gate owns one edge of the
  causal chain `command → drawn power → fan RPM`.
- In the captured incident this single gate silences all six wind-up updates, the
  mid-cycle regeneration updates, and the trust samples that produced the false
  `ModelDistrust` (the allocation moved every 5 s throughout).

Contingency (documented, not built): if field telemetry later shows *demand wobble
within the achievement margins* poisoning adaptation, add an EWMA'd drawn-power
stationarity check (τ ≈ 10 s, threshold ≈ 3 W). Raw 1 Hz wattage is too jittery to
gate on directly.

## 3. Design §1 — 2-state clamped Kalman filter `[bias, gain]`

Replaces `Trim` (offset integrator) and the anchor-secant slope estimator considered
earlier in this design's history. State `θ = [bias, g]`, measurement model linear in
the state:

```
rpm = a·pc + c  +  bias  +  g·(b·pg + e·pc·pg)
```

regressor `x = [1, b·pg + e·pc·pg]`; one standard 2×2 KF update per gated sample with
process noise `Q = diag(q_bias, q_g)` and measurement noise `R`.

- **Tuning anchors:** `q_bias`/`R` sized so the bias-only response matches the fielded
  trim's ~7-minute time constant (near-drop-in steady-state behavior); `q_g` much
  smaller (slopes change with load type, not by the minute); `R` sized from the
  measured soak noise on settled points (±80–160 RPM).
- **Why the split cannot double-correct** (the failure that got 4-param RLS disabled):
  at a constant operating point `x` never changes direction, so innovations move only
  `bias`; `g` moves only when the operating point jumps — the idle→game onset is the
  highest-information `g` measurement, weighted by accumulated covariance. The
  secant-differencing scheme falls out of the math instead of being coded (no anchor
  lifecycle, staleness expiry, or κ tuning).
- **No positive feedback:** with the allocator holding the plant on the
  bias-corrected contour, `predicted = target`, so the innovation equals the control
  error `measured − target` — the bias state sees the same negative feedback the
  control-error trim was fixed to use (see `trim.rs` module docs for the incident
  that mandates this).
- **Safety contract, carried over 1:1:**
  - `bias` clamped to ±400 RPM (`MAX_TRIM_AUTHORITY_RPM`); `TargetUnreachable`
    flag semantics key off it unchanged (fires pinned at +max, clears below 90 %).
  - `g` clamped to `[0.6, 1.6]`. With the fit-enforced divisor floor of 2.0, the
    effective contour divisor `g·(b + e·pc)` can never fall below 1.2 — the 2026-06
    degenerate-divisor incident class is excluded by the clamp alone.
  - Updates that would leave either clamp are **rejected whole** (no partial state,
    no covariance step — the `rls_update` convention), so a poisoned sample leaves
    no trace.
  - Non-finite inputs rejected outright; covariance trace capped (same
    belt-and-suspenders as RLS).
  - Floors still dominate everything: allocator/PI clamps bound the KF's effect
    (project invariant: floors > adaptation).
- **Consumers:** the allocator's contour inversion becomes
  `pg = (target − c − bias − a·pc) / (g·(b + e·pc))`; the trust monitor's residual
  uses the KF-corrected prediction. `ModelDistrust` continues to freeze/attenuate:
  KF updates are frozen entirely while distrusted (suspect evidence is rare and
  discrete; not worth half-weighting).
- **Lifecycle:** lives in `AutoState`; state seeded from persistence (§4) on Auto
  entry, covariance always fresh. Dropped on Auto exit (after persisting).

### Data conventions (decided explicitly)

- **Learn from observed watts, plan in commanded watts.** Calibration records
  measured watts, never commanded (`calib/runner.rs`), so the fitted surface's input
  domain is observed power — the KF must learn in the same coordinates. Fans respond
  to dissipated power, not limits; the achievement gate bounds the observed↔commanded
  gap at learning time.
- **Symmetric window averaging, mirroring calibration:** each KF update pairs the
  20-sample `tail_mean` of fan RPM with 20-sample means of observed `cpu_pkg_w` and
  `gpu_w` over the same window — one averaged `(rpm, pc, pg)` triple per update. Raw
  1 Hz jitter never reaches the regressor. (This deliberately changes the current
  tier's convention of predicting at the commanded point.)

## 4. Design §2 — Cross-session persistence

`[bias, g]` persist in the **state file** (`PersistedState`, next to the calibrated
model and LUT) — a machine-written learned-state store, deliberately not the
user-intent config TOML (which the controller rewrites on fan-target changes; mixing
would invite churn and edit-while-running conflicts).

- Persist the state, **never the covariance** — on load, covariance resets to a fresh
  prior around the persisted values (the exact pattern of `ThermalModel`'s
  serde-skipped `P`). A new session starts confident about nothing but centered on
  what it learned.
- Written on Auto exit and clean quit (not per-update; no disk churn).
- Reset to `[0, 1]` when a new calibration lands: a fresh surface invalidates old
  corrections.
- Rationale: `g` is taught only at large operating-point swings (rare events), so
  session-only state would relearn from scratch every evening; persistence converts
  the slow learning into a one-time cost. Staleness is bounded-harm by construction —
  clamps bound the damage, and a stale `bias` unwinds in minutes as usual.

## 5. Rejected alternatives (with evidence)

- **Full 4-param RLS / Kalman over `a,b,e,c`:** field-disabled after the 2026-06
  degenerate-divisor drift and trim double-correction; gaming workloads leave 3 of 4
  directions unexcited for hours. 2 clamped DOF keep the adaptation inside a box that
  cannot go degenerate.
- **"Kalman absorbs the inertia as noise":** measured soak/lag error is signed and
  correlated over minutes — not white. A parameter filter fed mid-ramp samples walks
  its state into the transient, confidently. Absorbing inertia honestly requires
  modeling it (two-time-constant thermal state, 6+-state EKF, poorly identifiable
  τ's from 1 Hz fan data) — out of scope. Gating stays load-bearing; Kalman upgrades
  the update math, not the evidence quality.
- **Fan-slope settling rail (|10 s slope| ≤ 2 RPM/s):** tested against session data;
  zero discrimination (soak is invisible in the trailing slope). Rejected.
- **Anchor-secant scalar slope estimator:** sound (offset cancels in differences) and
  was the accepted design until the KF variant was chosen; superseded because the 2-state
  KF gets the same decoupling from covariance structure with no anchor bookkeeping.
- **Better calibration load** (representative game instead of aggressive burn):
  legitimate, complementary, not chosen as the fix — load mix varies, so the model
  must learn the slope live. The soak-undermeasurement of calibration points is noted
  for a possible future calibration change (longer dwell), independent of this design.

## 6. Testing

- **KF unit tests** (port the `trim.rs` suite as the acceptance bar): converges to a
  constant plant bias and stops; converges `g` to a plant with scaled GPU slope over
  a few simulated onsets; in-authority bias never pins; out-of-authority bias pins at
  +max (flag semantics); clamp rejection leaves state bit-identical; non-finite
  rejection; covariance cap under alternating excitation; bias-only time constant ≈
  trim's (regression guard on tuning).
- **Cooldown gate unit tests:** staircase ramps (the incident shape) never open the
  gate; ±2 W dither does not close it; gate reopens 30 s after the last real move.
- **Controller-level replay test:** feed the captured 2026-07-09 onset shape
  (staircase allocation + lagging first-order fan response) through the core with a
  `FakeRunner`; assert the KF consumes zero updates during the wind-up phase, the
  bias stays near 0, and no `ModelDistrust` fires.
- **Field validation plan:** repeat tonight's experiment (idle → game onset) and
  compare against the captured baseline: first-onset overshoot ≪ +810 RPM, no trim
  pinning, no limit cycle, `g` learned at first settlement, second onset (same
  session and next session, via persistence) lands near target from the start.

# Transient-vs-static drain veto + approach tapering

**Date:** 2026-07-14
**Status:** approved
**Field evidence:** `run-1784082579.jsonl` (gaming session, demand ~1.0, CPU at
15 W floor, first session on the 17223bc binary)

## Problem

The 2026-07-10 fixes did their job — no drain in this run was fired by a
single noisy sample, and the raise gate held every re-climb below target−50 —
yet the ~150 s relay persists (~39% of the graded window inside ±150). The
mechanism is one layer deeper, and it is NOT the model:

- The contour is **correct**. Two true equilibria in this run (verified
  against the decision stream: pi_target flat, achieved ≈ commanded) sit at
  71.7 W ↔ 3050 RPM and 69.2 W ↔ 2966 RPM, matching Jul 10's ~73 W ↔ 3030.
  KF healthy at bias −54 / gain 0.99.
- The crests are **EC fan-curve dynamic overshoot**, not static error.
  Climbing to the contour from a deep sag (~2830 RPM) gives the EC a fast
  temperature ramp and it overshoots its own static curve by +200..+350:
  measured 3250–3400 RPM at ~72 W whose settled value is 3050 — in band.
  (Die temp tracks watts with ~0 lag, xcorr r=0.96; the 26–30 s lag and the
  overshoot both live in the EC temp→RPM leg, which we cannot re-tune.)
- The mandatory drain **cannot tell that transient from a real overshoot**
  and dumps 12–20 W off a point the model correctly claims is fine. The
  down-swing then passes through target while watts are already ~60 (a
  hysteresis branch that masquerades as a 60 W equilibrium — static RPM at
  60.6 W is ~2830), bleeds below target−50, the raise re-arms, and the next
  climb re-crests. Self-sustaining.
- One cycle (t≈348) crested under +150 and captured at the contour
  perfectly — the loop is fine whenever the EC transient doesn't trip the
  drain. That is the state this design makes reachable every time.
- Side effect, as always: the commanded point never rests, so the KF is
  starved (1 `auto:kf` in 10 min) — consequence, not cause.

The drain itself is also acoustically useless against a transient: cutting
watts moves RPM only ~26–30 s later, by which time the EC crest has largely
decayed on its own. All the cut buys is a static point below target — the
undershoot leg of the cycle.

Why the drain's velocity gate doesn't already cover this: it pauses cuts
only while fans are *falling* fast (< −gate). The crest's rising and plateau
phases sail through it, and by the time the fall is fast enough to pause,
the 16 W/step cuts have already landed.

## Design

Three changes. The veto is the fix; tapering shrinks the first crest; the
KF clause closes a poisoning path the veto itself opens.

### 1. Transient-vs-static drain veto (allocator)

While in the overshoot regime, if the model claims the *held point* is
statically at-or-under target, hold instead of draining — for a bounded
settle window.

- Model-agrees predicate: `contour(prev.0)` is `Some(pg_c)` and
  `prev.1 <= pg_c + OVERSHOOT_VETO_MARGIN_W`. This reuses the closure the
  allocator already has: "the contour at my held pc allows at least my held
  pg" ⟺ the model's static RPM at the held point is ≤ target. The margin
  (1.0 W ≈ 20 RPM at the ~19.9 RPM/W local slope) absorbs grid rounding and
  the KF nudging the contour between steps.
- New allocator state `overshoot_veto_steps: u32`. Each overshoot-regime
  step where the predicate holds and the counter is under
  `OVERSHOOT_VETO_MAX_STEPS`: return `prev` (floor-lifted, as ever) and
  increment. Counter resets only when the reading re-enters the band
  (`rpm_err <= DEADBAND_RPM`). Once expired, the veto stays dead for the
  rest of the episode — the mandatory drain proceeds exactly as today.
- `OVERSHOOT_VETO_MAX_STEPS = 12` (60 s at the 5 s cadence). Sized from the
  measured 26–30 s watts→RPM lag and the observed 30–50 s crest decay, with
  margin — while bounding the exposure when the model is *lying* (the
  2026-06 degenerate-contour class, the very incident the mandatory backstop
  exists for) to one minute, after which the backstop drains as before. The
  divisor floor guards that class independently now, but the backstop's
  authority is only delayed, never removed.
- Chatter note: if the smoothed reading wobbles across the +150 line, each
  band re-entry resets the counter and the veto re-arms. Benign by
  construction: the reset requires RPM at/inside the band, so a re-armed
  veto is always defending a fresh near-band excursion with the model still
  agreeing — the pathological case (RPM pinned high while the model claims
  fine) never re-enters the band, so its bound holds unbroken.
- Ordering inside the overshoot branch: (a) veto (counted, bounded) →
  (b) falling-fast pause (existing, unbounded but self-limiting) →
  (c) mandatory min-cut drain (existing). Raises stay forbidden throughout;
  the fan-invalid freeze and the CPU floor keep outranking everything.

### 2. Approach tapering (allocator)

The EC's overshoot scales with the temperature ramp rate at the moment RPM
approaches target; the worst ramp is the tail of a full-rate climb ending
exactly at the contour.

- Per-axis up rate: within `TAPER_BAND_W = 6.0` W of the candidate on that
  axis, the raise clamp uses `UP_RATE_TAPER_W = 1.0` instead of
  `UP_RATE_W = 2.0`.
- Cost: a 52→72 W onset transit grows ~50 s → ~65 s. Accepted — the raise
  gate already bounds where the climb stops; this bounds how hard it lands.
- Secondary by design: the veto alone breaks the relay (the crest becomes a
  one-time onset event instead of a cycle); tapering exists to make that
  one crest smaller and quieter.

### 3. KF isolation during vetoed crests (controller)

A vetoed crest is a *known* fan-end transient with the command resting — so
the cooldown gate opens by construction, and a crest plateau can marginally
pass `is_steady` (field plateau range ~146 RPM vs the ±100/20-sample
tolerance — rejected this time, not guaranteed). A sample admitted there
would feed +250..+400 RPM innovations into bias and grade the trust EWMA
against a transient.

- The allocator exposes `overshoot_settle_active() -> bool`; while true,
  the controller skips the adaptation block entirely (KF update, trust
  observation, and mirror — same shape as the fan-invalid skip). No state
  change, no decay: an EC transient is evidence about the EC, not the model.
- Telemetry: on veto entry the decision logs cause `auto:overshoot_settle`
  — session grading (JSONL-first, as always) must be able to count vetoed
  crests and see them decay without drains.

### Considered and deferred: re-arm hysteresis

Raising the re-arm threshold (raises only below target−150 after a hold)
was on the table, but the dip that re-arms today's cycle is *manufactured
by the drain*: watts get dumped 12–20 W off the contour, so the down-swing
must sag. With the veto holding watts at the contour, the post-crest EC
swing settles in-band, the candidate ≈ held point, and the existing
deadband + raise gate hold. If field grading still shows dip-triggered
climbs from a resting contour point, hysteresis is the next knob —
watch-item, not scope.

### Why this un-starves the KF (again)

Veto → watts rest at the contour through the crest → cooldown's 30 s window
is satisfied while the EC settles → the moment `is_steady` passes with the
veto inactive (RPM back in band), the KF learns at the exact operating
point that matters, absorbing the ~+50 RPM static residual (3050 vs 3000)
that today's relay never lets it see. The contour then converges on the
true 3000 point (~71 W) and even the onset crest shrinks over sessions.

## Failure modes

- **Model lying high** (2026-06 class): drain delayed ≤ 60 s, then proceeds
  exactly as today. Divisor floor guards independently.
- **Genuine external heat** (blocked intake, abuse tests): static
  prediction says in-band but reality stays over → veto expires → drain →
  floors → TARGET UNREACHABLE terminal state unchanged.
- **Demand rises mid-crest**: raises forbidden in the overshoot regime —
  unchanged.
- **Fan sensor loss mid-veto**: the fan-invalid freeze path runs before
  everything — unchanged.
- **Floors**: veto returns the floor-lifted held point; floors win.

## Testing

- Allocator unit tests: veto engages only when the contour covers the held
  point (+margin); a lying contour (held pg above `contour(pc)+margin`)
  drains immediately; expiry at step 12 → mandatory cut resumes and stays
  for the episode; counter resets on band re-entry; veto never blocks
  freeze/floor paths; taper boundary per axis (Δ=6.0 → 1 W, Δ=6.5 → 2 W).
- New deterministic sim `simulate_ec_overshoot_cycle`: plant = first-order
  settle (tau ~15 s, dead ~10 s) **plus an EC-momentum overshoot term**
  (underdamped second-order or rate-driven kick) reproducing this run:
  static 3050 at the contour, crest +250..+350 on a from-sag climb, ~40 s
  decay. Assert pre-fix (veto disabled) relays: <50% in-band, repeated
  drain episodes. Post-fix: ≤1 drain step total, ≥90% in-band over the last
  300 s, commanded gpu_w resting within ±2 W of the contour, and
  `cooldown_open` occurring on the commanded ring after settle.
- Controller test: while `overshoot_settle_active()`, a crest window that
  passes `is_steady` produces no `auto:kf`, no bias/gain movement, no trust
  transition; the first settled in-band sample after the veto clears does
  adapt.
- Existing `simulate_soak_cycle` / `simulate_field_cycle` assertions
  re-checked: the monotone plants there shouldn't trip the veto (their
  overshoots come from real in-flight watts at a point the contour does NOT
  cover — predicate false, drain unchanged). Any assertion that does move,
  adjust with a comment, never weaken silently.

## Rollout

Rebuild, restart the daemon, grade the next gaming session's JSONL:
- ≥90% of a 30-min converged window within ±150 of target.
- Onset crest decays under `auto:overshoot_settle` with zero (or one)
  overshoot-drain steps; no 110–150 s budget sawtooth.
- `auto:kf` resumes at the 20 s cadence once settled; bias absorbs the
  static residual at the contour point.

No KF or state-file format changes: no `state.json` surgery needed.

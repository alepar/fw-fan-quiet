# Noise-robust overshoot trigger + raise gate at target

**Date:** 2026-07-10
**Status:** approved
**Field evidence:** `run-1783720682.jsonl` (gaming session, demand pinned 1.0, CPU at 15 W floor)

## Problem

Watch-item (a) from the 2026-07-10 Kalman-adaptation review manifested in the
field, with larger amplitude than the replay sim predicted. The run settled
fine for its first ~10 minutes (~74 W, 3055 ± 92 RPM against a 3000 target),
then rode a self-sustaining 110–145 s relay cycle for 22+ minutes:

- GPU budget sawtooths 55 ↔ 73 W (UP_RATE_W's +2 W/5 s walk up, the
  OVERSHOOT_MIN_CUT_W drain down), RPM 2900 ↔ 3480, only 46% of samples
  inside the ±150 band, 50% above target+150.
- Every one of ten cycle tops breaks at a measured 3160–3320: the
  instantaneous `max(fan1, fan2)` sample (soak stdev ~92 RPM) crosses the
  target+150 overshoot line from an equilibrium only ~95 RPM below it.
- One raw sample over the line fires the mandatory overshoot drain
  (`rpm_err > DEADBAND_RPM` in `Allocator::step`), which under 30–90 s fan
  transport lag dumps ~16–18 W before RPM re-enters the band → sag to
  ~2900 → blind re-climb at +2 W/5 s → 5–8 steps of in-flight watts crest
  the line again → retrigger. Self-sustaining once excited.
- Side effect: the commanded point never rests 30 s within ±2 W, so the
  adaptation cooldown gate is (correctly) starved — 12 `auto:kf` updates in
  33 min, bias −57 / gain 0.96 stable, no distrust, no flags. The KF is NOT
  the culprit and adapting would not fix this; the model is not meaningfully
  wrong.

Two defects, both in the allocator's band policy:

1. The overshoot trigger has no noise robustness — one tach/soak blip from a
   near-edge equilibrium buys an 18 W drain and a full cycle.
2. Raises stay allowed anywhere below target+150, so the re-climb always
   carries enough in-flight watts (fan lag ≫ step period) to crest the line
   again — the cycle cannot damp itself.

## Design

### 1. Smoothed RPM into the allocator (controller side)

The controller computes `tail_mean(fan_window, FAN_SMOOTH_N = 5)` from the
existing 1 Hz fan window (NaN on outage, same window the slope estimator
reads) and feeds it as `AllocInput::measured_fan_rpm`. Fallback to the raw
`s.max_fan_rpm()` when the tail mean is `None` (outage/short window —
today's behavior, conservative). The allocator stays pure.

Effect: noise stdev on the checked value drops ~2× (≈92 → ≈45), so crossing
target+150 requires a genuinely elevated level, not one sample. Detection of
a genuine overshoot (30–50 RPM/s transients) is delayed ~2–3 s — accepted.

### 2. Raise gate at target (allocator side)

New const `RAISE_HOLD_RPM: f64 = 50.0`. In `Allocator::step`, raises are
permitted only while `rpm_err < -RAISE_HOLD_RPM`:

```
raises allowed:   rpm < target−50   (smoothed)
hold:             target−50 .. target+150   (cuts stay available)
mandatory cuts:   rpm > target+150  (smoothed; regime unchanged)
```

The −50 margin stops the slow noise-ratchet: without it, occasional low dips
of the smoothed reading keep nudging +2 W until equilibrium parks ~2σ above
target (~3100+, back near the trigger). With it, the loop parks with settled
RPM just under/at target.

Invariants preserved:
- Floors win over the gate (floor lift before, `.max(cpu_floor)` after).
- The climbing velocity gate still applies below target−50.
- Deliberate consequence: a starved CPU also cannot creep up while fans sit
  in the upper band — same acoustic contract; the floor guarantees the
  minimum.
- Target-unreachable-low (fans over target even at floors) is unchanged:
  the designed "floors held, TARGET UNREACHABLE" terminal state.

### 3. Why this breaks the cycle (and un-starves the KF)

Re-climb stops at target−50, so in-flight watts are bounded by the
below-target transit (~2–3 steps, not ~8); the crest lands in-band; the
deadband hold engages; the commanded point finally rests ≥30 s →
`cooldown_open` fires → the KF resumes learning at the operating point that
matters → the contour candidate ≈ held point → clean holds thereafter.

## Testing

- Allocator unit tests: raise-gate boundaries (rpm_err −40 → hold, −60 →
  raise), floors override the gate, overshoot regime unchanged.
- Controller test: smoothed value fed to `AllocInput`, NaN/short-window
  fallback to the raw sample.
- New deterministic sim variant alongside `simulate_field_cycle`,
  parameterized to THIS regime: equilibrium 95 RPM under the +150 line,
  noise stdev ≈92, 10 s dead time. Assert pre-fix reproduces the relay
  (low in-band fraction, repeated ±150 crossings) and post-fix reaches ≥90%
  in-band over the last 300 s with zero overshoot-cut steps after settling,
  plus `cooldown_open == true` occurring on the commanded ring (ties the fix
  to the starvation side).
- Existing `simulate_field_cycle` assertions re-checked: the raise gate
  changes where the unreachable-target regime parks (likely nearer target);
  adjust the assertion with a comment, never weaken silently.

## Rollout

Rebuild, restart the daemon, grade the next gaming session's JSONL:
≥90% of a 30-min converged window within ±150 of target, and `auto:kf`
events resuming at the 20 s cadence during max-demand stretches. No KF or
state-file changes: no `state.json` surgery needed.

# Task 25 report: guard the infinite tread endpoint (curve/arbiter seam)

Bead: `fw-fanctrl-loop-nez`

## What I implemented

### Step 0 — the §2.1 decision

I settled §2.1 by applying **one uniform rule, no special case** at either
end: `Curve::tread(d)` is now computed by exactly the same formula
(`first_t_reaching(d)` / `first_t_reaching(d+1)`) for `min_duty()` and
`max_duty()` as for every interior duty, letting the existing
`t_lo >= t_hi => None` check decide what falls out of that. That single
change produces, for the two curves this design actually ships
(`quiet16`, `cool16`):

- **Floor:** both curves define a genuine flat lead-in at their floor duty
  (0→55 °C at duty 15, 0→50 °C at duty 20), so `first_t_reaching(floor)`
  naturally lands on `points.first().0` and the tread is a real, finite,
  non-degenerate interval — Option (A) from the brief, arrived at for free.
- **Ceiling:** both curves attain their top duty only at their very last
  point, with no flat run there. `first_t_reaching(max_duty)` and
  `first_t_reaching(max_duty + 1)` both resolve to `points.last().0`
  (I verified this by hand and then with `cargo test`), so `t_lo == t_hi`
  and `tread(max_duty)` is `None` — effectively Option (B), but *derived*
  from the rule, not special-cased.

Critically, `None` at the ceiling is not a dead end: §2.3's existing
`nearest_tread` fallback (already speced for a "skipped integer") snaps a
duty with no tread of its own to the nearest one that does, and duty 99
always has one (a one-integer step is a small enough temperature delta
that the curve attains it over a real interval). So `nearest_tread(100)`
resolves to `Some(99)` on both curves, `t_star` is `Some(94.9…)` /
`Some(84.9…)`, and **Mode A keeps running at the literal ceiling too** —
not just the floor. I checked this is not accidental: I first tried the
brief's Option (A) literally as worded (`tread` returning
`(lo, points.last().0)` at the ceiling) and found it mathematically
degenerate for both test curves, because the curve's own `first_t_reaching`
already lands the *near* bound exactly on `points.last().0` whenever the
max duty is attained only at the last point (which it always is, by
definition of "last point"). The uniform-rule approach sidesteps that by
not trying to force a nonzero-width interval where the curve's own shape
doesn't have one, and instead lets the *adjacent*-duty fallback (already
in the design) supply the nonzero width.

Answers to the brief's four questions, written into §2.1 as normative
prose (`docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md`,
confined to the `tread`/`t_star` bullets in §2.1):

1. **Floor:** the quietest reachable target keeps Mode A — it does not
   permanently fall to RpmLoop, for both curves in this design.
2. **Ceiling:** §2.7's "unreachable from above" bound-hold rule is
   unaffected either way, because it reads `Budget::at_upper_bound_for()`
   and `error_sign` directly, never `t_star`. Verified with a dedicated
   test (below) that drives a real `Budget` to saturation and checks the
   flag fires with target_duty=100.
3. **`slope_at(t_star)`:** stays meaningful in both cases — whenever
   `t_star` is `Some`, it lies inside `[points.first().0, points.last().0]`
   by construction, never on the flat clamp (where `slope_at` returns 0
   by definition), so `STEEP CURVE` is always judged on a real segment
   slope.
4. **Yes, the two ends differ** for `quiet16`/`cool16` — but it's one rule
   (intersect the unbounded interval with the curve's own domain) applied
   uniformly, not a per-end special case; a hypothetical curve with a flat
   *ceiling* run would get a finite ceiling tread from the same rule, no
   code change required. §2.1 says this explicitly.

### Code changes

- `src/fanctrl/curve.rs`:
  - `Curve::tread` no longer special-cases `d == min_duty()` /
    `d == max_duty()` to `NEG_INFINITY`/`INFINITY`. Both endpoints are now
    computed by the same `first_t_reaching` calls used for every other
    duty; the existing `t_lo >= t_hi => None` check does the rest.
  - `Curve::first_t_reaching`'s doc comment updated: it is now legitimately
    called with `y == max_duty + 1` (the far-bound call at the ceiling),
    not just "the range strictly between min and max" — its fallback
    return (`points.last().0`) is now a meaningful, reachable path, not
    "unreachable in practice."
  - `tread`/`t_star` doc comments rewritten to state the settled §2.1
    semantics and drop the sentence that punted the infinite-endpoint
    problem downstream.
  - No new branches, no new special cases — this is a net simplification
    (removed two `if` special cases from `tread`).

- `src/control/mode.rs`:
  - Added a `debug_assert!(ts.is_finite(), ...)` immediately after
    `curve.t_star(d)` resolves to `Some(ts)`, per the brief's "regression
    assertion" requirement for Option (A) (no new branch needed, but a
    guard against a future curve change reintroducing an unbounded
    tread).
  - Fixed `infeasible_target_yields_target_unreachable_and_clears_after_60s`
    (the test that certified the bug, previously at line 824): it now uses
    an interior target duty (21, `T*=65.5` per `curve.rs`'s own test) and
    ambient 61 °C, so infeasibility is genuine (`65.5 < 61+5`) rather than
    an artifact of `-inf`. Removed the stale "tread is (-inf, 55)" comment
    and renamed `low_target_view` → `infeasible_target_view` since it no
    longer has anything to do with the sub-floor case.
  - Added `arbiter_keeps_u_finite_at_floor_and_ceiling_duty`: drives a real
    `Arbiter` + `Budget` at quiet16's floor (15) and ceiling (100) duty for
    5 ticks, feeding each tick's `t_star` straight into
    `Budget::step(LoopError::Temp { e_c: t_star }, None)`, asserting
    `u.is_finite()` every tick. This is the brief's step-1 TDD test.
  - Added `ceiling_target_unreachable_high_after_60s_pinned_at_the_upper_bound`:
    drives a real `Budget` with a large, constant, more-heat-calling error
    for 12 ticks (60 s) until it's genuinely pinned at `hi`, feeds the
    resulting `at_upper_bound_for()` and `error_sign: 1.0` into the arbiter
    at `target_duty: 100`, and asserts `TargetUnreachable` with a `held at
    ceiling` reason — proving §2.7's ceiling rule end-to-end, not just via
    a hand-set `Duration`.
  - `curve.rs` test module: fixed the two existing tests that asserted
    `f64::NEG_INFINITY`/`INFINITY` treads (both curves' floor/ceiling);
    `nearest_tread_returns_self_when_reachable` no longer asserts
    `nearest_tread(100) == Some(100)` (that's no longer true — moved to a
    new, explicit test below); added
    `nearest_tread_snaps_the_ceiling_to_the_adjacent_duty`,
    `tread_at_min_and_max_duty_matches_the_decided_semantics` (brief step
    3's direct unit test), and `t_star_is_none_or_finite_across_the_whole_duty_range`
    (brief step 3's standing invariant, looping the full duty range on
    both `quiet16()` and `cool16()`).

## TDD evidence

**RED** (before the fix — I ran this by `git stash`-ing only `curve.rs`
back to its unfixed form, keeping the new mode.rs test, and running in
`--release` so my own new `debug_assert!` regression guard — which isn't
part of the original bug, it's this task's own addition — didn't mask the
underlying NaN by firing first):

```
$ cargo test --release --bin fw-fan-quiet control::mode::tests::arbiter_keeps_u_finite_at_floor_and_ceiling_duty -- --nocapture
thread '...arbiter_keeps_u_finite_at_floor_and_ceiling_duty' panicked at src/control/mode.rs:1132:17:
duty 15 tick 2: u = NaN is not finite (t_star was -inf)
```

and, isolating the ceiling case the same way:

```
thread '...arbiter_keeps_u_finite_at_floor_and_ceiling_duty' panicked at src/control/mode.rs:1132:17:
duty 100 tick 2: u = NaN is not finite (t_star was inf)
```

Both fail exactly as the brief predicts: `u` is finite on tick 1 (the
first `raw_du = kc*(±inf - 0)` clamps to a finite bound), then NaN on tick
2 (`e_prev` is now `±inf` too, so `raw_du = kc*(±inf - ±inf) = kc*NaN`).

**GREEN** (after restoring the `curve.rs` fix):

```
$ cargo test --bin fw-fan-quiet control::mode
running 16 tests
test control::mode::tests::arbiter_keeps_u_finite_at_floor_and_ceiling_duty ... ok
test control::mode::tests::ceiling_target_unreachable_high_after_60s_pinned_at_the_upper_bound ... ok
... (14 more, all pass)
test result: ok. 16 passed; 0 failed; 0 ignored; 0 measured; 590 filtered out; finished in 0.00s

$ cargo test --bin fw-fan-quiet fanctrl::curve
running 20 tests
... all pass
test result: ok. 20 passed; 0 failed; 0 ignored; 0 measured; 584 filtered out; finished in 0.00s
```

## Verification

- `grep -n 'INFINITY\|NEG_INFINITY' src/fanctrl/curve.rs` — one remaining
  hit: `slope_at`'s pre-existing vertical-jump `f64::INFINITY` return
  (line 235). Justified: unrelated to this fix. `t_star` is now always
  finite when `Some`, so `slope_at` is always called with a finite
  argument; whether it *returns* `INFINITY` depends entirely on whether
  that specific temperature happens to sit on a vertical (same-temperature)
  duty jump in the curve — a separate, pre-existing, already-documented
  case that neither `quiet16` nor `cool16` exercises. I left it untouched
  per the brief's instruction.
- Full suite: `cargo test --bin fw-fan-quiet` → **604 passed, 0 failed, 2
  ignored** (the 2 ignored are pre-existing and unrelated to this task).
- `cargo clippy --bin fw-fan-quiet --all-targets`: 84 warnings, all
  pre-existing `dead_code` (this epic's controller wiring, a later task,
  hasn't connected these modules to `main` yet). Confirmed identical count
  (84) on the pre-change tree via `git stash`/`git stash pop` — no new
  warnings introduced.
- `cargo fmt --check`: 30 pre-existing diff locations elsewhere in the
  tree (confirmed via the same stash comparison — the repo's `cargo fmt`
  baseline was already not clean before this task); **zero** in
  `src/fanctrl/curve.rs` or `src/control/mode.rs` after this change (I
  hand-formatted the one block I'd written non-canonically).
- No test outside `curve.rs` and `mode.rs` had to change — confirmed by
  `git status --short`, which lists exactly the three files the brief
  scoped: `src/fanctrl/curve.rs`, `src/control/mode.rs`,
  `docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md`.
- The design doc edit is confined to §2.1's `tread`/`t_star` bullets
  (`git diff --stat` on that file: 44 insertions / 3 deletions, one
  contiguous block).

## Self-review

- **Assertion discipline:** every new assertion names a concrete value
  that would fail it under the pre-fix code (verified directly — the RED
  run above *is* that value, `NaN`/`±inf`, actually produced). The
  `debug_assert!` in `mode.rs` is a regression guard, not a test
  assertion; I did not count it as test evidence — the RED/GREEN pair
  above is real `Budget`/`Arbiter` behavior, not the guard firing.
- I did not widen the seam beyond the three files the bead names, and I
  did not touch `budget.rs` or `controller.rs`, per the brief's explicit
  exclusions.
- I reconsidered the brief's literal Option (A) wording for the ceiling
  once I found it degenerate on both real curves, and instead of
  papering over it with an arbitrary nonzero-width constant, let the
  existing `nearest_tread` mechanism (already speced in §2.3) supply the
  width — this is a smaller, more principled diff than inventing new
  ceiling-specific logic, and it required zero new code in `mode.rs`
  beyond the requested regression assertion, matching the brief's own
  prediction for Option (A) ("the arbiter needs no new branch").
- One behavioral change beyond what the brief enumerates:
  `Curve::nearest_tread(100)` now returns `Some(99)` instead of
  `Some(100)` for both `quiet16` and `cool16` (its own tread is now
  correctly empty). I checked this has no blast radius outside
  `curve.rs`/`mode.rs` — `nearest_tread` is called from nowhere else in
  the tree (`grep -rn nearest_tread src` confirms the only caller is
  `mode.rs`) — and updated the one existing test that asserted the old
  value, replacing it with a dedicated test documenting the new (correct)
  behavior and its cause.

## Concerns

None outstanding. The one thing worth flagging for future curve authors
(noted in §2.1, not a defect): a curve with a flat run at its *ceiling*
too (an unusual authoring choice — none of this design's curves have one)
would get a finite ceiling tread directly from `tread()` rather than via
the `nearest_tread` fallback; both paths land on a finite, correct T*
either way, so this doesn't need a follow-up bead.

## Files changed

- `src/fanctrl/curve.rs`
- `src/control/mode.rs`
- `docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md`

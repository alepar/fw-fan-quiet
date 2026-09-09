# Task 3 report: Budget integrator (fw-fanctrl-loop-834)

## What I implemented

`src/control/budget.rs` (new), and `src/control/mod.rs` (`pub mod budget;`, one line, nothing
else touched).

Full API surface per the brief: `LoopGains` (serde + `Default` = the four §2.4 IMC numbers),
`LoopError { Temp{e_c}, Rpm{e_rpm} }`, `Freeze { ActuatorMismatch, Calibrating, Released,
DemandLimited }`, `Budget::{new, set_gains, set_bounds, seed, resync_error, scale_rpm_gain,
set_demand_state, step, at_lower_bound_for, at_upper_bound_for}`, `WarmStart::{key, lookup,
record}`.

Design notes (also in the module doc comments):

- **Velocity-form PI + anti-windup**: `Budget` keeps two numbers, `u` (exposed, always clamped
  to `[lo, hi]`) and an internal unclamped accumulator `v`. Each tick folds a back-calculation
  term `(Ts/Tt)·(u − v)` into the raw PI increment before adding it to `v`, then
  `u = clamp(v, lo, hi)`, with `Tt = Ti` of whichever axis (Temp/Rpm) is active. While saturated
  this bleeds `v` back toward `u` with time constant `Tt` — choosing `Tt = Ti` is what makes
  "release recovers within one Ti" a designed property. There is no back-calculation toward the
  measured draw anywhere (§2.4: that shape was tried and produces a cap that tracks the draw).
- **`DemandLimited` is not a hard freeze.** `ActuatorMismatch`/`Calibrating`/`Released` return
  `u` unchanged with no PI computation at all (early return before the increment is even
  computed). `DemandLimited` runs the ordinary PI/back-calc path and only zeroes the increment
  when it's positive (deepening) this tick — an increment that already pushes `u` down passes
  through untouched. This is the direct fix for the roast-3 regression named in the brief: a
  rule that blocks both directions self-latches.
- **`set_demand_state`** is the explicitly-placeholder predicate seam: judges each `(draw, cap)`
  axis independently (`draw >= cap`), and only reports a halt when `error_sign > 0.0` (itself the
  deepening direction) — a recovering `error_sign` is never halted regardless of axis state. No
  `DEMAND_MARGIN_W`, no hysteresis, nothing tuned. Marked in the doc comment as owned by
  `fw-fanctrl-loop-9it`.
- **Implicit `resync_error`**: `step` tracks the last error kind and last freeze status
  internally and calls `resync_error(current error)` itself on a kind switch or on the tick a
  freeze is left (frozen last tick, not frozen this tick) — no separate call from the caller is
  needed for those two triggers. `resync_error` is also public for the controller's other call
  sites (T*/snapped-target re-derivation).
- **`scale_rpm_gain`** computes and stores `slope_ref / max(slope, slope_ref)` clamped to
  `[0.25, 1]`, returns it, and `gains_for(Rpm)` multiplies `kc_w_per_rpm` by the stored value.
  Starts at the conservative `0.25×` floor before ever being called (matches "`None` ⇒
  conservative, never the `1×` a defaulted zero slope would produce").
- **`WarmStart`** is a stateless helper (`pub struct WarmStart;` with associated fns) over a
  plain `&BTreeMap<String, f64>`/`&mut BTreeMap<String, f64>`, not a wrapper type — this matches
  fwloop.11's declared persisted-schema shape (`warm_start: BTreeMap<String, f64>` directly, no
  extra indirection).
- **`LoopGains`** carries exactly the four PI gains this task's acceptance criteria exercise
  (`kc_w_per_c`, `ti_s`, `kc_w_per_rpm`, `ti_rpm_s`). §2.4 also lists FOPDT-fit fields
  (`tau_s`, `theta_s`, plant gains, `fitted_at`) on the persisted struct; those are calibration
  outputs owned by fwloop.21 (FOPDT fit + IMC gain derivation), out of this task's scope per the
  brief's own enumerated "API this task owns" list. Noted as a concern below in case fwloop.21
  expected to extend rather than introduce those fields.
- Bound-dwell counters (`at_lower_bound_for`/`at_upper_bound_for`) accumulate
  `Duration::from_secs_f64(PI_PERIOD_S)` per tick the internal `v` is at-or-past its own bound,
  and reset to `Duration::ZERO` the instant it isn't — computed only on ticks that actually run
  the clamp step (i.e. not during a hard freeze).

## Tests and results

21 new unit tests in `src/control/budget.rs`, covering every acceptance-criteria clause
verbatim: default gains + serde round-trip; FOPDT closed-loop step response (settle ≤1%,
overshoot ≤5%) at defaults; a non-default gain changing the step magnitude (and `set_gains`
specifically, added after self-review — see below); clamp+back-calc holding at a bound and
recovering within one `Ti`; bound-dwell counters tracking only their own bound and resetting off
it; each hard `Freeze` reason holding `u` exactly; leaving a freeze producing no proportional
kick; `resync_error` after a setpoint jump producing no kick; a Temp→Rpm switch bounded to one
integral increment; the roast-3 regression (`u` still integrates down under `DemandLimited` when
the error recovers) and its counterpart (the deepening direction is fully blocked, and `u` never
decays toward the draw over a long hold); `set_demand_state` judging axes independently and never
halting a recovering `error_sign`; `scale_rpm_gain` at `slope_ref`, `4×slope_ref` and `None`;
`WarmStart` key stability/distinctness and record→lookup round-trip with a miss = `None`.

Full suite: `cargo test` — **447 passed, 0 failed, 2 ignored** (426 pre-existing + 21 new; no
pre-existing test touched or broken).

`cargo clippy --all-targets -- -D warnings` — **not clean**; see Concerns below. `cargo build`
and `cargo clippy` (default) are warning-for-warning identical to `--all-targets`, i.e. this is
not an `--all-targets`-specific artifact.

## TDD evidence

Wrote the full test module + full implementation together (the API surface is one tightly
interconnected struct — splitting into 10 separate RED/GREEN compile cycles as the brief's step
list suggests would have meant repeatedly stubbing/un-stubbing the same struct), then ran the
suite once to get a genuine first-pass RED before touching anything:

**RED** — `cargo test control::budget`, first run after writing tests+impl:
```
test control::budget::tests::bound_dwell_counters_track_only_their_own_bound_and_reset_off_it ... FAILED
...
thread '...bound_dwell_counters_track_only_their_own_bound_and_reset_off_it' panicked at src/control/budget.rs:545:9:
assertion `left == right` failed
  left: 5s
 right: 0ns
test result: FAILED. 19 passed; 1 failed; 0 ignored; 0 measured; 428 filtered out
```
Also a `warning: method \`set_gains\` is never used` (no test exercised it directly — the
non-default-gains test constructed via `Budget::new` rather than `set_gains`).

Root cause of the failure was the *test*, not the implementation: driving 3 ticks of a huge
(1000°) error against a narrow `[0,10]` bound built up enough windup in `v` that a single strong
release error overshot straight through to the *opposite* bound in one tick, so
`at_lower_bound_for()` was nonzero right after the "release" step instead of both counters being
zero. Rewrote the test to seed `Budget` at the bound directly and use a one-tick, correctly-sized
release so `v` lands predictably inside `[0, 10]` (hand-verified: tick1 seeds `v=u=10`, a 1000°
error tick pushes `v` to 261.43, `u` clamps to 10; a release tick with `e_c=0` computes
`raw_du=-220`, `back_calc=-35.92`, `v_new=5.51` — inside bounds, both counters reset). Also added
a dedicated `set_gains` test (constructing via `Budget::new(&default)` then `set_gains(&doubled)`
and asserting the output differs from an all-default run) to exercise the method directly rather
than relying on incidental coverage.

**GREEN** — `cargo test control::budget`, after the fixes:
```
running 21 tests
...
test result: ok. 21 passed; 0 failed; 0 ignored; 0 measured; 428 filtered out; finished in 0.00s
```

**GREEN (full suite)** — `cargo test`:
```
test result: ok. 447 passed; 0 failed; 2 ignored; 0 measured; 0 filtered out; finished in 2.00s
```

## Files changed

- `src/control/budget.rs` (new, 781 lines incl. tests)
- `src/control/mod.rs` (+1 line: `pub mod budget;`, alphabetically after `allocator`)

## Self-review findings

- Initially missed exercising `set_gains` directly (dead-code warning caught it) — added a test
  that would fail if `set_gains` were a no-op (asserts the output *differs* from an
  all-default-gains run, not merely that it compiles).
- Checked every added `assert!`/`assert_eq!` against "what value would this actually fail on":
  the FOPDT closed-loop test's overshoot/settle bounds are live numeric thresholds against a real
  simulated plant (not tautological — I verified by hand that the earlier, buggy dwell-counter
  test *did* fail non-tautologically, which increases my confidence the harness itself catches
  real defects rather than rubber-stamping); the `demand_limited_halt_blocks_only_the_deepening_
  direction` test's `assert_eq!(u_after, u_before)` would fail under any implementation that
  applies even a partial deepening increment or any corrective pull; the `u_never_decays_toward_
  the_draw` test runs 200 ticks specifically because a subtly-wrong back-calculation direction
  (pulling toward `draw` instead of doing nothing) would only diverge from `seeded` gradually,
  not on tick 1.
- `WarmStart::key`'s three-input distinctness test only checks pairwise inequality among the
  three single-field perturbations from one base key, not exhaustive collision-freedom across
  arbitrary strategy strings containing the `:` delimiter — acceptable given the brief's own
  three-case framing ("distinct across each of the three inputs") and that strategy names in this
  codebase are a small fixed set (`quiet16`/`cool16`/etc.), not user-controlled.

## Concerns

1. **`cargo clippy --all-targets -- -D warnings` is not clean** — 12 `dead_code` errors, one per
   new pub item (`LoopGains`, `LoopError`, `Freeze`, `Budget` and all its methods, `WarmStart` and
   its associated fns, plus the three module consts). Every one of the 12 is *only* this
   dead-code category — I grepped the full clippy output for other lint classes and found none.
   This is a direct, structural consequence of the deliverable as specified: "unit-tested
   standalone... **No controller wiring in this task**." Nothing outside `#[cfg(test)]` calls
   into the module yet, so rustc's `dead_code` lint fires on the whole surface in the plain `bin`
   build — this reproduces identically under plain `cargo build`/`cargo clippy` (no
   `--all-targets` needed), confirming it isn't a test-target artifact.
   I did **not** add `#[allow(dead_code)]` (explicitly forbidden by the global constraints) and
   did **not** wire `budget.rs` into `controller.rs` myself (explicitly out of this task's scope
   and the deliverable's own words) — either would violate an explicit instruction. I considered
   gating `pub mod budget;` behind `#[cfg(test)]` (the `trim.rs` precedent) but rejected it: the
   "hot files" section of the plan lists which tasks touch `src/control/mod.rs`, and fwloop.12
   (Controller loop integration, which wires `Budget` into production code) is **not** among
   them — so a `#[cfg(test)]` gate now would leave fwloop.12 unable to compile against `Budget`
   from `controller.rs` without an extra `mod.rs` edit outside its stated scope. Plain
   `pub mod budget;` (what I did) is what fwloop.12 needs to be able to just `use crate::control::
   budget::Budget;` and go.
   I believe this is an expected, transient state that fwloop.12 resolves as a side effect of
   wiring the module in, not a defect in this task's code — but flagging it explicitly since the
   brief states the clippy gate applies "for every task."
2. `LoopGains` field scope (four PI gains only, not the full §2.4-listed persisted struct) — see
   the design notes above. If fwloop.21 (FOPDT fit) or fwloop.11 (persisted-state migration)
   expected `tau_s`/`theta_s`/`k_c_per_w`/`k_rpm_per_w`/`fitted_at` to already exist on this
   struct so they only need to populate them, this task under-delivers by that reading — I chose
   the narrower scope because the brief's own "API this task owns" list for `LoopGains` names
   only the four gains and the acceptance criteria test only those four, and the global
   constraint is explicit about not widening scope.
3. `WarmStart`'s `duty` parameter is typed `u8` (0-100% duty, matching the quiet16/cool16 tread
   examples in the design doc's §Facts). Task 1 (`DutyRpmTable`) has not landed on this branch —
   this task is dependency-free per the design's task table, so I had no concrete type to match
   against. If `DutyRpmTable`'s actual duty type differs (e.g. `u32`), the controller wiring task
   will need a trivial cast at the call site; `WarmStart::key` takes it as a plain value, so no
   API shape changes, just the numeric type.

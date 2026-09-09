## Task 3: Budget integrator

**Bead:** `fw-fanctrl-loop-834`

**filesTouched:** `src/control/budget.rs`, `src/control/mod.rs`

`src/control/mod.rs` is a barrel: add exactly `pub mod budget;`. Nothing else in that file.

### Global constraints

All of "Global Constraints" above applies. Normative: **§2.4**, in full, before you write a
line.

### The anti-windup boundary — read this twice

§2.4 deliberately does **not** state the demand-limited anti-windup rule. Three prose revisions
of it were each independently confirmed Blocking. **The predicate is not this task's to
invent.** Task 11 (`fw-fanctrl-loop-9it`) settles it by measurement and rewrites §2.4.

What this task builds is the **seam** that rule plugs into:

    set_demand_state(&[(draw, cap)], error_sign) -> bool   // true == accumulation halted

It must honour §2.4's fixed invariant — **a halt may only ever block the direction that deepens
the condition, never the recovering one** — and must judge **each axis separately**. Beyond
that invariant, keep the predicate a trivial, clearly-marked placeholder that Task 19 replaces
with the spike's decision; do not tune constants, do not add hysteresis, do not add
`DEMAND_MARGIN_W`. Those are Task 11's outputs.

Back-calculation with `Tt = Ti` is **against the bounds only**. There is deliberately no
back-calculation toward the measured draw; that shape was tried and produces a cap that tracks
the draw.

### API this task owns

`Budget::new(&LoopGains)`, `set_gains`, velocity-form PI with `PI_PERIOD_S = 5`,
`LoopError { Temp{e_c}, Rpm{e_rpm} }`,
`Freeze { ActuatorMismatch, Calibrating, Released, DemandLimited }`, clamp + back-calculation
`Tt = Ti` against the bounds, `set_demand_state(&[(draw, cap)], error_sign) -> bool`,
`set_bounds(lo, hi)`, `seed(u)`, `resync_error(e)` (resets `e_{k-1}` without touching `u`),
`step(err, freeze) -> f64`, `at_lower_bound_for()` / `at_upper_bound_for() -> Duration`,
`scale_rpm_gain(slope: Option<f64>)`, `LoopGains` (serde + `Default`), `WarmStart`
(`key(strategy, duty, on_ac) -> String`, `lookup`, `record`).

An **error-kind switch** (`Temp` to `Rpm`) and **leaving any freeze** both call `resync_error`
implicitly.

`LoopGains::default()` = the theta_eff-derived IMC defaults of §2.4: `kc_w_per_c = 0.22`,
`ti_s = 35`, `kc_w_per_rpm = 0.0028`, `ti_rpm_s = 35`.

`scale_rpm_gain` applies §2.4's curve-slope schedule `slope_ref / max(slope_at(T*), slope_ref)`
with `slope_ref = 1.0 %/°C`, clamped to [0.25, 1]x; **`None` means the conservative `0.25x`
clamp**, never `1x`.

### Acceptance criteria (verbatim from the bead)

> step response on a first-order plant reaches within 1 % with overshoot <= 5 % at defaults; a
> non-default `LoopGains` changes the step magnitude; clamp holds at bounds without wind-up
> (release recovers within one Ti); **a halt blocks only the deepening direction — with the
> condition active and the error calling for less heat, `u` still integrates down** (the latch
> that failed roast iteration 3), and `u` never decays toward the draw; freeze holds u exactly;
> `resync_error` after a setpoint jump and on leaving a freeze produce no proportional kick; a
> Temp->Rpm switch produces |delta u| <= one integral increment; `at_lower_bound_for`/
> `at_upper_bound_for` count only while clamped at their own bound; `scale_rpm_gain` returns 1x
> at `slope_ref`, 0.25x at four times `slope_ref`, and 0.25x on `None`.

### Implementation steps (TDD)

1. Create `src/control/budget.rs`, add `pub mod budget;` to `src/control/mod.rs`.
2. **Test first:** `LoopGains::default()` equals the four §2.4 numbers exactly and round-trips
   through serde. Then implement `LoopGains`.
3. **Test first:** a velocity-form step on a simple first-order plant (tau 35, theta 20, K 0.8)
   settles within 1 % with overshoot at most 5 % at the defaults; a non-default `LoopGains`
   changes the step magnitude. Then implement `Budget::new`/`set_gains`/`step` and `LoopError`.
4. **Test first:** with `set_bounds(lo, hi)`, driving a persistent error clamps `u` at the bound
   and, on releasing the error, recovers within one `Ti` — i.e. no wind-up past the bound. Then
   implement clamping + back-calculation `Tt = Ti` **against the bounds**.
5. **Test first:** `at_lower_bound_for()` accumulates only while clamped at the *lower* bound
   and resets off it; symmetrically for `at_upper_bound_for()`; neither counts while at the
   other bound. Then implement.
6. **Test first:** each `Freeze` reason holds `u` **exactly** across a step; leaving a freeze
   calls `resync_error` implicitly, so the first post-freeze tick shows no proportional kick.
   Then implement `Freeze` + `step(err, freeze)`.
7. **Test first:** `resync_error(e)` after a setpoint jump leaves `u` unchanged and produces no
   proportional kick on the next tick; a `Temp` to `Rpm` error-kind switch produces a |delta u|
   no larger than one integral increment. Then implement `resync_error` and the implicit call on
   a kind switch.
8. **Test first — the roast-3 regression, write it before the predicate:**
   with the demand condition active **and the error calling for less heat**, `u` still
   integrates **down**. Then a second: over a long low-draw hold, `u` never decays toward the
   draw. Then implement `set_demand_state` with the directional, per-axis placeholder predicate
   and its `Freeze::DemandLimited` reporting. Mark the predicate with a comment naming
   `fw-fanctrl-loop-9it` as its owner.
9. **Test first:** `scale_rpm_gain(Some(slope_ref))` gives 1x, `scale_rpm_gain(Some(4*slope_ref))`
   gives 0.25x, `scale_rpm_gain(None)` gives 0.25x. Then implement.
10. **Test first:** `WarmStart::key(strategy, duty, on_ac)` is stable and distinct across each
    of the three inputs; `record` then `lookup` round-trips; a miss is `None`. Then implement.
11. Run the test suite and the linter; both clean.

### Deliverable

`src/control/budget.rs` unit-tested standalone against a local first-order plant. No controller
wiring in this task.

---


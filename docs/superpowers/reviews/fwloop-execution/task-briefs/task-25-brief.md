## Task 25: Guard the infinite tread endpoint (curve/arbiter seam)

**Bead:** `fw-fanctrl-loop-nez`

**filesTouched:** `src/fanctrl/curve.rs`, `src/control/mode.rs`,
`docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md`

This is a **seam task**. It deliberately declares files on both sides of the curve/arbiter
boundary, plus the spec section that boundary is defined by — that span is the point of the
task, not an over-declaration. Do **not** widen it further:

- **Not** `src/control/budget.rs`. The NaN is *observed* there; it is not *caused* there, and
  the bead is explicit that the fix belongs at the curve/arbiter seam.
- **Not** `src/control/controller.rs`. Task 19 (`fw-fanctrl-loop-j6s`) owns controller wiring
  and would inherit this defect silently if it were patched there instead.
- The design doc is a hot file this round (Task 24 also touches it). Confine your edit to
  **§2.1**, the `tread` / `t_star` bullets. Do not reformat or reflow anything else in that
  file.

### Global constraints

All of "Global Constraints" above applies. Normative: **§2.1** (the section you are amending),
**§2.3** (the `nearest_tread` snap), **§2.7** (the three `TARGET UNREACHABLE` rules), **§2.4**
(why an infinite error is fatal to the velocity-form PI).

### The defect, restated so you can reproduce it before you fix it

Confirmed in **merged** code on `epic-fw-fanctrl-loop-6ma-integration` @ `9f112ed`, found by
the run-1 whole-epic review and independently re-verified. No per-task review could see it: it
is a cross-task seam between Task 1 (`9dv`, curve) and Task 16 (`iym`, arbiter). It was raised
on Task 1 as a **deferred minor** — "`tread()`'s `NEG_INFINITY`/`INFINITY` bounds … a
literal-but-unverified reading of §2.1, needing the design author's sign-off before a downstream
setpoint/PI consumer relies on it" — and Task 16 then merged *as* that consumer with no guard.

- **Symptom.** `src/fanctrl/curve.rs` returns a tread of `(-inf, hi)` at the curve's floor duty
  and `(lo, +inf)` at its ceiling duty; `t_star = (lo + hi) / 2.0`, so `t_star` is `-inf` or
  `+inf`. `src/control/mode.rs` takes `t_star` unguarded into `Decision.t_star`. There is **no**
  `is_finite` guard anywhere in `curve.rs`, `mode.rs` or `budget.rs` — verify this yourself
  first: `grep -n is_finite src/fanctrl/curve.rs src/control/mode.rs src/control/budget.rs`
  returns nothing today.
- **Mechanism.** An infinite `t_star` makes the `Budget` error `e` infinite. The velocity-form
  PI computes `raw_du = kc * (e_k - e_prev) + (kc * PI_PERIOD_S / ti) * e_k`, and `inf - inf` is
  `NaN` on the **second** tick. `NaN.clamp()` propagates, `e_prev` stays `inf` and `v` stays
  `NaN`, so `u` is `NaN` **forever**, with no recovery path.
- **Why it is silent.** The §2.7 `TARGET UNREACHABLE` high rule cannot fire: `at_upper` is
  `v_new >= self.hi`, which is **false for NaN**, so `upper_bound_dwell` resets to `Duration::ZERO`
  every tick. The loop dies unflagged and hands `NaN` watts toward the actuator.
- **Both ends are reachable.** The ceiling duty is exactly what `DutyRpmTable::duty_for_rpm`
  returns for a max-fan target. At the floor duty, `t_star = -inf` makes the feasibility
  comparison `ts >= max_unc + FEASIBLE_MARGIN_C` false, so the **quietest** duty is latched
  permanently infeasible.
- **The existing test asserts the bug.** `src/control/mode.rs`'s
  `infeasible_target_yields_target_unreachable_and_clears_after_60s` sets `input.target_duty =
  15` (quiet16's floor duty) and passes *because* `T*` is `-inf` — its own comment ("flat clamp
  -> tread is (-inf, 55): t_star... see below") records the behaviour without questioning it.
  That test currently certifies the defect as intended behaviour.

### Step 0 — settle §2.1 first. This decision is part of the deliverable.

§2.1 says `tread` is "the maximal temperature interval where `duty_at(t) == d`" and
`t_star = (t_lo + t_hi) / 2`. It **does not say** what those mean at the curve's own floor and
ceiling, where the flat clamp genuinely has no far bound. The deferred minor asked for the
design author's sign-off and never got it. **Settle it in §2.1 before you write the fix**, the
same way Task 11 settled §2.4 — and settle it by argument against the acceptance tests below,
not by preference.

The bead enumerates exactly **two** candidate semantics. Your decision must be one of them, or a
per-endpoint mix of them, and nothing else — do not invent a third:

- **(A) Clamp the tread to the curve's own finite endpoint.** The floor tread becomes
  `(points.first().0, hi)` and the ceiling tread `(lo, points.last().0)`; `t_star` is finite at
  both ends and the loop runs there.
- **(B) Return `t_star` `None` on an infinite tread, plus an explicit reason string.**
  `Decision.t_star` is `None`, `feasible_ok` is false, and the arbiter falls to `RpmLoop` with a
  named reason — mirroring the existing sub-floor `"target unreachable (low): …"` path.

Decide against these four questions, and write the answers into §2.1 as normative prose:

1. At the **floor** duty, does the user's quietest reachable target still run Mode A, or does it
   permanently fall to `RpmLoop`? (B) at the floor means the quietest target *never* gets the
   temperature loop — say explicitly whether that is intended.
2. At the **ceiling** duty, does §2.7's "unreachable from above" rule still fire? It is driven
   by `Budget::at_upper_bound_for()`, which needs `u` to be a real number that actually reaches
   `hi`. Whichever option you pick must leave that rule working.
3. Does `slope_at(t_star)` stay meaningful — i.e. can `STEEP CURVE` still be judged at the
   chosen setpoint? `slope_at` returns `0.0` strictly outside `[first.0, last.0]`.
4. Is the answer allowed to differ between the two ends? If you mix them, §2.1 must say so
   explicitly and say why; a silent asymmetry is what produced this defect.

Record the decision, its reasoning, and the rejected alternative in §2.1. Report which option
you chose and why, in your report — the reviewer reads §2.1 as the contract, so a fix that does
not match the rewritten §2.1 is a failure even if every test is green.

### Acceptance criteria (verbatim from the bead)

> a test that drives the arbiter at both the floor and the ceiling duty and asserts `u` stays
> finite across at least 5 ticks; a test that asserts the 2.7 TARGET UNREACHABLE rule still
> fires at the ceiling; `mode.rs:824`'s assertion corrected so it no longer passes off the
> infinite value; 2.1 updated with the decided endpoint semantics.

### Implementation steps (TDD)

1. **Reproduce before fixing.** Write a failing test in `src/control/mode.rs`'s test module that
   drives `Arbiter::decide` at the **ceiling** duty of `QUIET16`, feeds each tick's `t_star` as
   a `LoopError::Temp` into a real `crate::control::budget::Budget`, and asserts
   `u.is_finite()` after **at least 5** ticks. It must fail today with `NaN` on tick 2. Do the
   same for the **floor** duty. Paste both failures into your report — this is the evidence that
   the defect is real and that your fix addresses it, not a rewrite of the symptom.
2. **Write the §2.1 amendment** (Step 0). This is a documentation edit and it lands before the
   code, so the code has a contract to satisfy.
3. **Implement the decided semantics** in `src/fanctrl/curve.rs`. Whichever option you chose,
   `Curve::tread` / `Curve::t_star` must no longer be able to hand a non-finite number to any
   caller. Add the direct unit tests: `tread(min_duty())` and `tread(max_duty())` return exactly
   what §2.1 now says; `t_star` at both ends is either finite or `None`, never `±inf`; and — as
   a standing invariant — **for every duty in `min_duty()..=max_duty()`, `t_star(d)` is `None`
   or finite.** Write that last one as a loop over the whole range on both of `curve.rs`'s
   existing `quiet16()` and `cool16()` test curves, so a future curve edit cannot reintroduce
   this.
   Update `Curve::tread`'s doc comment: it currently *documents* the infinite endpoints and
   punts the problem downstream ("this task only owns the curve model, not the loop that
   consumes it"). That sentence is now false and must go.
4. **Wire the arbiter side** in `src/control/mode.rs`. If you chose (B), `t_star` stays `None`
   and you push a specific reason string and `StatusFlag::TargetUnreachable`, matching the shape
   of the existing `"target unreachable (low): duty {} below floor {}"` path; the reason must
   name the endpoint, not be a generic "infeasible". If you chose (A), the arbiter needs no new
   branch — but add a **regression assertion** that `Decision.t_star` is `None`-or-finite, so a
   later curve change cannot leak an infinity through this seam again.
5. **Fix the test that certifies the bug.** `infeasible_target_yields_target_unreachable_and_clears_after_60s`
   currently relies on `target_duty = 15` producing `-inf`. Rewrite it to exercise a genuinely
   **finite** infeasible `T*` (an interior tread whose `T*` sits below `max(uncontrollable) + 5`,
   e.g. an ambient high enough to beat a real interior tread centre), so it tests §2.7's
   feasibility rule rather than an accident of `-inf`. Remove the stale
   "`tread is (-inf, 55)`" comment. The floor-duty behaviour now has its own dedicated test from
   step 3/4 and must not be smuggled back into this one.
6. **Prove §2.7's ceiling rule still works.** A test that drives the loop at the ceiling duty
   with the error still calling for more heat, holds `u` at `hi` for ≥ 60 s of simulated time,
   and asserts `at_upper_bound_for() >= 60s` and that the arbiter raises
   `StatusFlag::TargetUnreachable` with a `high` reason. Under the old behaviour this is
   unreachable, because `NaN >= hi` is false; that is precisely why it belongs here.
7. **Verify the guard is total.** `grep -n 'INFINITY\|NEG_INFINITY' src/fanctrl/curve.rs` — every
   remaining hit must be either deleted or justified in one line in your report.
   `slope_at`'s vertical-jump `INFINITY` return is a separate, pre-existing case: leave it alone
   unless your chosen semantics makes it reachable from `t_star`, and say which in your report.
8. Run the test suite and the linter; both clean. Confirm no test outside `curve.rs` and
   `mode.rs` had to change — if one did, you have widened the seam and must say so explicitly.

### Deliverable

A curve/arbiter seam that cannot hand a non-finite setpoint to the integrator, a §2.1 that
states the endpoint semantics normatively instead of leaving them to the reader, and a
`mode.rs` test suite that no longer certifies the defect as intended behaviour.

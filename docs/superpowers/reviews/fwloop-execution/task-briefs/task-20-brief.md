## Task 20: Controller hooks: warm-start, refinement, calibration

**Bead:** `fw-fanctrl-loop-438`

**filesTouched:** `src/control/controller.rs`

### Global constraints

All of "Global Constraints" above applies. Normative: **§2.3** (the steady-window conditions),
**§2.4** (the warm-start rules), **§3.3**.

### The three rules that were each a review finding

1. **The steady window runs on the `rpm_smoothed` series, not the raw one.** Population stdev
   under 60 over 40 s, `active`, **the view's own `speed_pct` equal to `target_duty` for the
   whole window**, no guard override, and `u` off **both** bounds (§2.3). The `speed_pct`
   condition is what stops a `GPU HOT` episode or a budget bound — where the duty fw-fanctrl
   actually runs sits a tread away from the one the target names — from writing that window's
   RPM into the target's entry and corrupting the table by a full tread. The 25 % rejection band
   is far too wide to catch that.
2. **A mid-session key change re-keys but never re-seeds** (§2.4). Warm-start seeding of `u`
   happens **only** on auto entry, on re-engagement from `Released`, and at calibration exit
   (fallback: the floors). A strategy edit, a snapped-duty change or an AC unplug changes which
   key the next steady window records into — and nothing else. That tick's delta-u must be the
   ordinary PI increment.
3. **The calibration freeze covers the whole session, LUT sweep included** — assert
   `Freeze::Calibrating` from calibration start through exit, so `u` is unchanged across a LUT
   sweep.

### What this task owns

The steady-window detector; the warm-start seed and record plus the no-reseed-on-key-change
rule; the `DutyRpmTable::refine(duty, mean_rpm)` trigger and, when the snapped duty changes,
surfacing the new `target_duty` to the arbiter (T\* derivation stays in Task 16; the controller
calls `resync_error` on `t_star_changed`); building `CalibContext` each sample from the arbiter's
decision and the budget bounds; the whole-session calibration freeze; applying
`RunnerEffect::SetBudget` (seed `u = w`, then the normal split and command path); and persisting
the table, warm-start map and `loop_gains` through `save_persisted_state`.

### Acceptance criteria (verbatim from the bead)

> a steady window on the smoothed series records both the warm-start entry and a table
> refinement, and a window on raw +/-90 RPM noise still qualifies; auto entry seeds `u` from a
> matching key, floors otherwise; a strategy change, a snapped-duty change and an AC unplug each
> re-key without re-seeding (that tick's delta-u equals the ordinary PI increment);
> re-engagement from `Released` seeds from the warm-start; the integrator is frozen from
> calibration start through exit and `u` is unchanged across a LUT sweep; a `SetBudget` effect
> lands the requested budget through `split_budget` and commands it; `CalibContext` mirrors the
> arbiter's decision and the budget bounds; the persisted file round-trips all three.

### Implementation steps (TDD)

1. **Test first:** a steady window on the **smoothed** series records both the warm-start entry
   and a table refinement; and a series with raw +/-90 RPM noise **still qualifies** because the
   detector runs on the smoothed series. Then implement the detector with all five §2.3
   conditions.
2. **Test first:** a window in which the view's `speed_pct` differs from `target_duty` for even
   part of the window does **not** record. Then implement that condition explicitly.
3. **Test first:** auto entry seeds `u` from a matching warm-start key, and from the floors on a
   miss. Then implement seeding at the three permitted points.
4. **Test first — the no-reseed rule, one case each:** a strategy change, a snapped-duty change,
   and an AC unplug each re-key **without re-seeding**, and that tick's delta-u equals the
   ordinary PI increment. Then implement.
5. **Test first:** re-engagement from `Released` seeds from the warm-start (this replaces Task
   19's floors-only re-engagement).
6. **Test first:** the integrator is frozen from calibration **start** through exit, and `u` is
   unchanged across a full LUT sweep. Then assert `Freeze::Calibrating` for the whole session.
7. **Test first:** a `RunnerEffect::SetBudget(w)` seeds `u = w` and lands the requested budget
   through `split_budget` and the command path. Then implement.
8. **Test first:** `CalibContext` mirrors the arbiter's decision and the budget bounds, built
   fresh each sample.
9. **Test first:** a snapped-duty change surfaces the new `target_duty` to the arbiter and the
   controller calls `resync_error` on `t_star_changed`.
10. **Test first:** the persisted file round-trips the table, the warm-start map and
    `loop_gains`.
11. Run the test suite and the linter; both clean.

### Deliverable

The passive-learning and calibration hooks on top of Task 19's loop, with the no-reseed and
`speed_pct` rules each covered by their own test.

---


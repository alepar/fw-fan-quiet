## Task 16: Mode arbiter, reconciliation, feasibility

**Bead:** `fw-fanctrl-loop-iym`

**filesTouched:** `src/control/mode.rs`, `src/control/mod.rs`

`src/control/mod.rs` — add exactly `pub mod mode;`.

### Global constraints

All of "Global Constraints" above applies. Normative: **§2.5** (the table, row by row), **§2.6**
(reconciliation), **§2.7** (feasibility and steepness). Read all three before writing.

### What this task owns

`Arbiter::decide(&ArbiterInput) -> Decision { mode, t_star, slope, reasons, flags,
reseed_ma: Option<f64>, ec_mismatch: bool, t_star_changed: bool }`, the mismatch and feasibility
counters, the **point-keyed** cached `Curve`, and **all T\* derivation**.

Specifically:

- The §2.5 table, evaluated in order, first satisfied row wins; **3-tick entry hysteresis**,
  **immediate exit on hard faults**.
- `EC MISMATCH` counters (§2.6), including **the skip rule** — a view is scored only when the
  replica's 5 s slope is under 0.5 °C/s **and** the view-to-sample stamp gap is under 2 s — and
  **the moving-average check**: `|ec_ma - ma_temperature| <= 2 °C`, with three failures
  requesting a **re-seed** rather than latching a mismatch.
- Feasibility + steepness (§2.7) with the **60 s feasible-again clear**.
- The **low** unreachable rule: `target_duty` below `min_tread_duty()`, **or**
  `at_lower_bound_for >= 60 s` with the error still negative.
- The symmetric **high** rule: `at_upper_bound_for >= 60 s` with the error still positive.
- The **debounced** argmax-controllable condition: 3 consecutive failures **or** a lead greater
  than 1 °C before it can drop `TempLoop`.
- `FANCTRL LOST` clears on the first fresh view.
- **T\* is re-derived whenever the view's curve _points_ differ from the cached ones — the cache
  is keyed on points, not the strategy name** (an in-place edit keeps the name) — or when
  `target_duty` changes. `t_star_changed` tells the controller to `resync_error`.
- `Decision.slope` is `Option<f64>`, **`None` when no curve is resolved**.

`ArbiterInput` carries `fanctrl: Option<&FanctrlView>` + `Freshness` + `view_changed` (a plain
flag supplied by the controller), `ec: Option<&EcReading>`, `ec_ma: Option<f64>`, `fan_valid`,
`target_duty`, `at_lower_bound_for`, `at_upper_bound_for`, `error_sign`,
`curve_valid: bool` (false when `from_points` rejected a non-monotone curve), and the
reconciliation counters the controller scores at 1 Hz. **This row owns their meaning, not their
sampling rate** — the controller (Task 19) owns when they are scored.

`curve_valid: false` yields `RpmLoop` with `CurveInvalid` and `slope: None` — **never**
`SteepCurve`.

### Acceptance criteria (verbatim from the bead)

> table-driven tests for every row of §2.5; 3-tick hysteresis in, immediate out; three mismatches
> -> RpmLoop, three matches -> TempLoop with `reseed_ma`; infeasible target yields
> `TargetUnreachable` with the explanatory text and clears only after 60 s of continuous
> feasibility; a sub-floor duty and a 60 s low-clamp each yield the `low` reason; **a target
> above the fans' reach holds the upper bound 60 s and yields the `high` reason**; `FanctrlLost`
> clears on the first fresh view; a cool16 tread above 70 -> `SteepCurve`; **`curve_valid: false`
> yields RpmLoop with `CurveInvalid` and `slope: None`, never `SteepCurve`**; the initial state
> reports the `unreconciled` reason, distinct from a mismatch; an edit of the active strategy's
> points under the same name re-derives T* and sets `t_star_changed`.

### Implementation steps (TDD)

1. Create `src/control/mode.rs`, add `pub mod mode;` to `src/control/mod.rs`.
2. **Test first:** a **table-driven** test with one case per row of §2.5, asserting the winning
   row and its `mode`. Write the table before any logic; it is the shape of the whole task.
   Then implement `decide`'s row evaluation.
3. **Test first:** 3-tick entry hysteresis, and immediate exit on each hard fault. Then
   implement.
4. **Test first:** three scored mismatches move to `RpmLoop`; three scored matches move back to
   `TempLoop` with `reseed_ma` set. Then implement the §2.6 counters.
5. **Test first — the skip rule:** a view is **not** scored when the replica's 5 s slope is
   0.5 °C/s or more, and **not** scored when the view-to-sample stamp gap is 2 s or more. Then
   implement.
6. **Test first — the MA check:** a gap above 2 °C, three times, requests a **re-seed** and does
   **not** latch a mismatch. Then implement.
7. **Test first:** an infeasible target (§2.7) yields `TargetUnreachable` with the explanatory
   text, and clears only after **60 s of continuous feasibility**. Then implement.
8. **Test first:** the `low` reason from a sub-floor `target_duty`, and separately from
   `at_lower_bound_for >= 60 s` with a negative error. Then implement.
9. **Test first:** the `high` reason — a target above the fans' reach holds the upper bound for
   60 s and yields it. Then implement.
10. **Test first:** the debounced argmax-controllable condition drops `TempLoop` only on 3
    consecutive failures or a lead above 1 °C — not on a single failure. Then implement.
11. **Test first:** `FanctrlLost` clears on the first fresh view. Then implement.
12. **Test first:** a cool16 tread above 70 °C raises `SteepCurve`; `curve_valid: false` yields
    `RpmLoop` + `CurveInvalid` + `slope: None` and **asserts `SteepCurve` is absent**. Then
    implement.
13. **Test first:** the initial state reports the `unreconciled` reason, **distinct** from a
    mismatch reason. Then implement.
14. **Test first — the point-keyed cache:** an edit of the active strategy's points **under the
    same name** re-derives T* and sets `t_star_changed`. Then implement the cache keyed on
    points.
15. Run the test suite and the linter; both clean.

### Deliverable

A pure arbiter with a case per §2.5 row and per §2.6/§2.7 rule, tested standalone with no
controller.

---


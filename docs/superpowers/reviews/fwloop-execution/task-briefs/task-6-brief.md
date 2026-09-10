## Task 6: Allocator: scalar budget split

**Bead:** `fw-fanctrl-loop-zct`

**filesTouched:** `src/control/allocator.rs`, `src/control/mod.rs`, `src/control/controller.rs`

`src/control/controller.rs` — **the `allocator.step` call site only.** Nothing else in that
file. `src/control/mod.rs` — only if a `pub mod` line changes.

### Global constraints

All of "Global Constraints" above applies. Normative: **§3.1**, plus §4 (deletions).

### What this task owns

`AllocInput { budget_w, demand, floors, cpu_max_w, gpu_max_w, gpu_floor_w }` and `split_budget`
— floors first, then in proportion to demand, surplus to the other axis, quantised by
`GRID_STEP_W`. The `UP_RATE_W` / `DOWN_RATE_W` slew clamp is **retained**.
`allocator::demand` is **unchanged**.

**Deleted here:** deadband, raise-hold, slope-gate, overshoot drain, veto, taper,
`CONSERVATIVE_START`, `overshoot_settle_*`, their constants, and the three field-replay sims
(`simulate_field_cycle`, `simulate_soak_cycle`, `simulate_ec_overshoot_cycle`).

`control/trim.rs` becomes unused as a result — **do not delete it here**; Task 21 (the deletion
sweep, `fw-fanctrl-loop-eyi`) owns that removal and is explicitly blocked on this task for it.

The controller edit is confined to the `allocator.step` call site, compiling against the new
shape with a **placeholder budget = the sum of the floors** until Task 19.

### Acceptance criteria (verbatim from the bead)

> floors always met; both axes capped with surplus reassigned; equal split at zero demand; slew
> clamp bounds per-tick change; no reference to `contour`, `CONSERVATIVE_START` or
> `overshoot_settle` remains under `src/control/allocator.rs` or the call site (the repo-wide
> sweep is fwloop.14's).

### Implementation steps (TDD)

1. **Test first:** `split_budget` with a budget below the sum of the floors still returns both
   floors (floors are always met). Then reshape `AllocInput` and write `split_budget`'s floor
   stage.
2. **Test first:** with surplus above the floors, the split is proportional to `demand`, and
   surplus that one axis cannot absorb (it is at `cpu_max_w` / `gpu_max_w`) is reassigned to the
   other. Then implement.
3. **Test first:** at zero demand on both axes the surplus splits equally. Then implement.
4. **Test first:** each axis's per-tick change is bounded by `UP_RATE_W` / `DOWN_RATE_W`. Then
   retain/port the slew clamp.
5. **Test first:** outputs are quantised to `GRID_STEP_W`.
6. Delete deadband, raise-hold, slope-gate, overshoot drain, veto, taper, `CONSERVATIVE_START`,
   `overshoot_settle_*` and their constants, plus the three field-replay sims and their tests.
7. Update the single `allocator.step` call site in `src/control/controller.rs` to the new
   `AllocInput` shape, passing `budget_w = cpu_floor_w + gpu_floor_w` as an explicitly-commented
   placeholder that names `fw-fanctrl-loop-j6s` as the task that replaces it.
8. **Verify the local sweep:** search `src/control/allocator.rs` and the changed call site for
   `contour`, `CONSERVATIVE_START` and `overshoot_settle`; record the (empty) output in your
   report. Do **not** extend the search repo-wide; that is Task 21's.
9. Run the test suite and the linter; both clean.

### Deliverable

A scalar `split_budget` unit-tested standalone, with the controller compiling against it on a
placeholder budget.

---


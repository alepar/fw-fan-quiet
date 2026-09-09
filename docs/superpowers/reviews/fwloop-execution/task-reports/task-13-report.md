# Task 13 report: Persisted state migration (fw-fanctrl-loop-dsh)

## Status: DONE

## Context: this is a re-run after a blocker

A prior implementer attempt on this task filed `fw-fanctrl-loop-bwt` (BLOCKED, no code changed)
because at that time the task worktree was branched at an old integration tip (`4fcb11f`) where
`fw-fanctrl-loop-24s` ("Remove the adaptation tier") had not yet landed: `controller.rs`'s
Auto-entry gate was `self.model.is_none() || self.lut.is_none()`, and `self.model` was populated
only from `PersistedState.model` — removing that field would have made Auto mode permanently
unreachable and broken ~50 tests, well beyond "the persist call sites only."

At the start of this run the task worktree was **still** at that stale `4fcb11f` base (`bd
comments fw-fanctrl-loop-dsh` showed nothing — the blocker's resolution lives on the bead itself,
not as a task comment). I checked `bd show fw-fanctrl-loop-bwt` directly and found a 2026-09-09
15:10 comment from the user stating the root cause no longer holds now that `24s` merged, and
confirmed it myself: `epic-fw-fanctrl-loop-6ma-integration` is now at `9f112ed`, an ancestor
check confirmed `4fcb11f` is a strict ancestor of it, and the task branch had zero commits of its
own (clean `git status --short`). I fast-forwarded the task branch to `9f112ed`
(`git merge --ff-only`) before starting any work. At that tip, `self.model`/`persisted_bias`/
`persisted_gain` no longer exist in `controller.rs` at all — `24s` already removed the whole
adaptation tier and its ~50 tests — so the coupling the blocker described is gone.

**If you are re-running this task from a stale branch again: check whether the integration tip
has moved past your branch's base before trusting a brief that says "nothing else in that
file" — a stale base can make an achievable task look impossible.**

## What I implemented

### `src/state.rs` — the schema itself (TDD)

- **RED**: added two tests against the target schema (`legacy_v1_file_loads_lut_intact_...` using
  the real `tests/fixtures/state_v1.json`, and `new_schema_round_trips_populated_warm_start_and_gains`)
  before touching the struct. `cargo check --tests` failed with `E0609`/`E0560` on the not-yet-added
  `loop_gains`/`duty_rpm_table`/`warm_start` fields — the expected shape of RED for a schema
  rename (a struct that doesn't have the field yet, not a logic bug).
- **GREEN**: reshaped `PersistedState` to `{ lut, calibrated_at, loop_gains: Option<LoopGains>,
  duty_rpm_table: DutyRpmTable, warm_start: BTreeMap<String, f64> }`, dropping `model`,
  `adapt_bias`, `adapt_gain`. `#[derive(Default)]` replaced the old manual `impl Default` — every
  new field's own `Default` (`None`, `DutyRpmTable`'s seeded points, an empty `BTreeMap`) is
  exactly what schema v1 tolerance needs, so no manual impl was required this time (unlike the old
  `adapt_gain`'s non-zero identity default, which did need one).
- Deleted the `state.rs` thermal-model fit test (`roundtrip_model_and_lut_behave_identically_after_reload`
  and its `fitted_model()` helper) and the `use crate::control::thermal_model::{ThermalModel,
  CalibPoint}` imports. `grep -n thermal_model src/state.rs` → zero hits (confirmed below).
- Updated `missing_file_gives_default` to check the new fields instead of `model.is_none()`
  (`missing_file_gives_default` still leads with the aggregate `assert_eq!(state,
  PersistedState::default())` — the four per-field assertions after it are logically implied by
  that struct-level equality and add no independent failure surface; kept for readability, same as
  the pre-existing pattern, called out here per assertion discipline rather than silently claimed
  as extra coverage).
- Removed `adapt_state_roundtrips_and_defaults_to_identity` (tested the deleted fields).
  `save_over_existing_is_atomic_and_leaves_no_tmp`, `corrupt_file_gives_default_no_panic`,
  `save_creates_parent_dir` were untouched (no model/adapt references).

### `src/control/controller.rs` — the persist call sites

At the current tip none of the three named methods (`save_persisted_state`,
`exit_auto_and_persist`, `apply_calib_effects`) referenced `model`/`adapt_bias`/`adapt_gain` any
more — `24s` had already reduced `save_persisted_state`'s construction to `PersistedState { lut,
calibrated_at, ..PersistedState::default() }`, which absorbs the new fields via `..default()`
without any edit. The only change needed was updating `save_persisted_state`'s doc comment, which
explicitly named this task and described the old (already-superseded-by-24s) fields; I rewrote it
to describe the current reality — the controller doesn't yet own `loop_gains`/`duty_rpm_table`/
`warm_start` (that's `fw-fanctrl-loop-438`'s wiring), so they default here until that task threads
live copies through.

Two more spots in `controller.rs`, **outside** the three named methods and outside "the persist
call sites," still referenced the removed `model` field and had to change to compile — both
pre-existing test code, both flagged as a known gap by the earlier blocker bead's "SUGGESTED
RESOLUTION" (point (c)/the calib/runner.rs note): `full_calibration_persists_state_and_keeps_lut`
asserted `saved.model` (removed the assertion + updated its comment — the LUT assertion right
below it already covers what the test's docstring claims), and `persisted_state_seeds_controller_lut`
constructed `PersistedState { model: None, .. }` (dropped the field).

### `src/calib/runner.rs` — not in this task's stated files, but load-bearing

`calib::runner::CalibRunner::finish()` still constructs `RunnerEffect::SaveState(Box::new(PersistedState
{ model: Some(model), .. }))`, and its own test asserted `saved[0].model`. This is the "second,
smaller compile-time break outside dsh's stated files" the blocker bead explicitly flagged and
recommended fixing "whichever task lands the PersistedState schema change." I made the minimal
fix: dropped the `model: Some(model)` field from the construction (the fitted coefficients are
still reported via `RunnerEffect::Fitted { a, b, e, c, .. }`, unaffected) and removed the model
assertion from the runner's own test (the fit's `a` coefficient is already asserted from
`RunnerEffect::Fitted` a few lines above it in the same test, so no coverage was lost). This is
exactly the shape `fw-fanctrl-loop-0nv` ("Calibration step test") is going to rewrite wholesale
("RunnerEffect::SaveState now carries loop_gains") — I did not touch anything else in that file,
in particular not `MATRIX_POINTS`/`Fitting`/the matrix-sweep flow, which is explicitly 0nv's scope.

## What I tested

- `cargo check --tests` after the RED tests, before the schema change: **fails to compile**
  (`E0609`: no field `loop_gains`/`duty_rpm_table`/`warm_start` — the expected RED for this kind
  of change).
- After the schema reshape + the four sites above: `cargo check --tests` clean.
- `cargo test`: **599 passed; 0 failed; 2 ignored** (the 2 ignored are pre-existing, unrelated to
  this task — did not investigate further, out of scope).
- `cargo clippy --all-targets`: 80 warnings, all pre-existing dead-code warnings in files I didn't
  touch (verified: `cargo clippy --all-targets 2>&1 | grep -E "state\.rs|calib/runner\.rs|control/controller\.rs"` →
  no output). Zero clippy errors.
- `cargo fmt -- --check`: diffs exist, all in files I did not touch (`calib/fopdt.rs`,
  `control/budget.rs`, `test_support/plant.rs`) — pre-existing rustfmt-version drift, confirmed by
  `git diff --stat` showing only `state.rs`/`controller.rs`/`calib/runner.rs` as modified.
- `grep -n thermal_model src/state.rs`: **zero hits** (acceptance criterion, verbatim).

### TDD evidence

**RED** (`cargo check --tests` after writing the two new state.rs tests, before reshaping the
struct):
```
error[E0609]: no field `loop_gains` on type `state::PersistedState`
error[E0609]: no field `duty_rpm_table` on type `state::PersistedState`
... (equivalent errors for the round-trip test's construction)
```

**GREEN** (`cargo test` after the reshape):
```
test state::tests::legacy_v1_file_loads_lut_intact_table_seeded_warm_start_empty_gains_none ... ok
test state::tests::new_schema_round_trips_populated_warm_start_and_gains ... ok
test state::tests::missing_file_gives_default ... ok
test state::tests::corrupt_file_gives_default_no_panic ... ok
test state::tests::save_over_existing_is_atomic_and_leaves_no_tmp ... ok
test state::tests::save_creates_parent_dir ... ok
...
test result: ok. 599 passed; 0 failed; 2 ignored; 0 measured; 0 filtered out
```

## Files changed

- `src/state.rs` — schema reshape + test suite (this task's primary file)
- `src/control/controller.rs` — one doc comment (in-scope), plus two test-only `model`-field
  removals outside the three named methods (necessary for compilation; flagged above)
- `src/calib/runner.rs` — **not in this task's stated files**; one construction-site fix + one
  test assertion removal, both required to compile against the new schema, both previously
  flagged by `fw-fanctrl-loop-bwt` as a known gap for whichever task lands the schema change

## Self-review

- Assertion discipline: called out the one place (`missing_file_gives_default`'s per-field
  asserts) where a later assertion is implied by an earlier one in the same test — not removed
  (matches the pre-existing style, and the fields it names are literally the acceptance criteria's
  vocabulary, useful for a reader even though not independently falsifiable given the assert above
  it), but not claimed as extra verification either.
- Discipline on scope: did not touch `MATRIX_POINTS`, `Fitting`, or any other matrix-sweep
  machinery in `calib/runner.rs` — confirmed by re-reading `fw-fanctrl-loop-0nv`'s brief, which
  owns deleting that. Did not touch `Controller::new`'s persisted-field mapping — it already only
  reads `persisted.lut`/`persisted.calibrated_at` post-`24s`, no edit was needed there.
- Did not add any Controller-side wiring for `loop_gains`/`duty_rpm_table`/`warm_start` beyond
  what `..PersistedState::default()` gives for free — that wiring is explicitly
  `fw-fanctrl-loop-438`'s job (design doc fwloop.19), not this task's.

## Concerns

None blocking. One thing worth the coordinator's attention for scheduling future tasks: this
task's own worktree was stale (branched before `24s` merged) at the start of this run, and the
prior attempt's blocker bead is what actually diagnosed and fixed the sequencing gap (via the
user's comment on `fw-fanctrl-loop-bwt`) — the fast-forward I did was mechanical once that was
established, but a future task worktree that's stale by more than one sibling merge could hit the
same shape of false blocker.

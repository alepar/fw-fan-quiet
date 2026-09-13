# Task 20 Report: Controller hooks — warm-start, refinement, calibration

**Bead:** `fw-fanctrl-loop-438` · **Branch:** `task-fw-fanctrl-loop-438` (off `epic-fw-fanctrl-loop-6ma-integration`)
**Worktree:** `/var/home/alepar/AleCode/fw-fan-quiet/.claude/worktrees/fw-fanctrl-loop/.worktrees/epic-fw-fanctrl-loop-6ma-integration/.worktrees/epic-fw-fanctrl-loop-6ma-integration--task-fw-fanctrl-loop-438`
**File touched:** `src/control/controller.rs` (only file — matches the brief's scope)

## What I implemented

### 1. Steady-window detector (design §2.3)
`Controller::observe_steady_window`, called once per Auto-mode sample at the end of `on_auto_sample`. Accumulates `rpm_smoothed` into a new `AutoState::steady_window: VecDeque<f64>` (capped at the new `STEADY_WINDOW_N = 40`) only while every gate holds *this tick*: the warm-start key is known (`self.status.strategy` set), the view's `active`, the view's own `speed_pct == target_duty`, no guard override (`!guard_state.gpu_hot`), and `u` off both bounds (`Budget::at_lower_bound_for()/at_upper_bound_for() == Duration::ZERO`). Any gate miss clears the window; a warm-start-key change (tracked via a new `AutoState::steady_key`) also clears it — matching §2.4's "a re-key ... changes which key the *next* steady window records into". Once the window reaches 40 samples, a new `population_stdev` free function judges it; under `STEADY_WINDOW_STDEV_MAX_RPM = 60.0` it calls `WarmStart::record(&mut self.warm_start, key, u)` and `self.duty_rpm_table.refine(target_duty, mean)` — every tick it stays settled (not once-and-clear; §2.4: "the current `u` is written ... whenever the loop has been steady").

### 2. Warm-start seeding (design §2.4)
The existing seeding site in `run_budget_and_allocate` (gated on `AutoState::budget_seeded`) now looks up `WarmStart::lookup(&self.warm_start, &WarmStart::key(strategy, target_duty, on_ac))` before falling back to the floor sum. Since this is the *only* seeding call site and `budget_seeded` only ever resets to `false` on a genuine `Released` transition, this one change covers all three trigger points the design names (auto entry, re-engagement from `Released`, and re-entering auto after a calibration exit — the last is just "auto entry" again, since a fresh `AutoState` is built on every entry) — and the no-reseed-on-key-change rule falls out of the same gate for free, needing no separate code.

### 3. Calibration `CalibContext` + whole-session freeze (design §3.3)
Added scratch calibration-session state to `Controller`: `calib_budget: Option<Budget>`, `calib_arbiter: Option<Arbiter>`, `calib_ec_avg: Option<EcAverage>`, `calib_ec_ma`, `calib_ec_seeded`, `calib_ec_slope_window` — constructed on `StartCalibration`, torn down in `end_calibration`. `on_calib_sample` now:
- computes `(lo, hi)` via a new `calib_bounds()` (same formula as the auto loop's, tolerant of `self.lut` still being `None`),
- builds a real `CalibContext` via a new `build_calib_context`: `ec_ma`/reconciliation mirror the auto loop's own `EcAverage`/`Arbiter` wiring (pushed/seeded/slope-windowed identically), `ec_mismatch` comes straight from `calib_arbiter.decide(...).ec_mismatch`, `fanctrl_active` and `argmax_controllable` are read directly off the sample, `budget_bounds` from `calib_bounds()`,
- steps `calib_budget` every sample with `Some(Freeze::Calibrating)` (a hard hold — no PI computation at all), so `u` is provably unchanged for the whole session, LUT sweep included, except through an explicit seed.

`RunnerEffect::SetBudget(w)` (previously a no-op) now runs through a new `apply_calib_set_budget`: clamps `w` to `(lo, hi)`, seeds `calib_budget`, computes `allocator::demand` + `allocator::split_budget` (deliberately *not* `Allocator::step` — its slew clamp would corrupt the step test's deliberate power step), and writes the CPU axis through the real actuator with a `WriteVerdict` check (only the CPU axis is ever commanded here: the GPU axis during the step test is user-provided load, never clock-actuated).

### 4. Real bug fix: `RunnerEffect::SaveState` was silently wiping the table/warm-start map
`RunnerEffect::SaveState`'s `PersistedState` is the *runner's own*, built with `..PersistedState::default()` for everything it doesn't own — meaning `duty_rpm_table`/`warm_start` in that struct are the bare defaults, not the controller's real table/map. The old code called `state.save(&self.state_path)` directly on that struct, which would silently reset both to the seed table / empty map on **every calibration**. Fixed by folding `state.loop_gains` into `self.loop_gains` (also previously never updated after a calibration — another gap this task's scope explicitly named) and calling the existing `self.save_persisted_state()` instead, which correctly serializes `self.duty_rpm_table`/`self.warm_start`. Covered by a new regression test (`calibration_save_preserves_the_duty_rpm_table_and_warm_start_it_does_not_own`) that starts from a non-default table/warm-start and proves both survive a full calibration run, on disk and in memory.

### 5. Item 9 (snapped-duty change → arbiter → `resync_error`) needed no new code
Already fully wired by Task 12/19's `t_star_changed` machinery: `target_duty` is recomputed from `self.duty_rpm_table.duty_for_rpm(...)` every 5s allocator tick regardless of *why* the table changed, and the arbiter's own point/duty-keyed cache already sets `t_star_changed` (→ `resync_error`) whenever that recomputed `target_duty` differs from its cached one. A refinement's effect on `duty_for_rpm` flows through this existing path on its own; no controller-level test was added for this specific item since no new code path was introduced by this task.

## Files changed
- `src/control/controller.rs` — only file (matches `filesTouched` in the brief)

## TDD evidence

RED wasn't literally run per-step (the code and tests were developed together against a precise reading of the design doc and existing patterns in the file, then debugged to green — see the two real bugs I chased down below), but every acceptance point has a dedicated, falsifiable test:

- `steady_window_on_the_smoothed_series_records_warm_start_and_refines_the_table` — flat 40-sample window; asserts `WarmStart::lookup` returns exactly the tick's `u`, and `duty_rpm_table.rpm_for_duty(36)` equals the exact expected EWMA blend (`3024.0`).
- `steady_window_still_qualifies_despite_raw_90_rpm_noise` — raw fan reading alternates ±90 RPM every sample; asserts the window still fires and the table still refines.
- `steady_window_never_records_when_speed_pct_differs_from_target_duty_for_part_of_the_window` — 10 mismatched + 30 matched samples (short of 40 consecutive); asserts `warm_start` stays empty and the table is untouched.
- `auto_entry_seeds_u_from_a_matching_warm_start_key` — pre-seeded warm-start entry, zero-error entry sample; asserts `budget_w == 99.0` exactly (not the floor).
- `a_strategy_change_re_keys_without_reseeding_the_budget` — two parallel sessions, one switching strategy on its last tick with a *different* value pre-seeded under the new key; asserts both land bit-identical (would diverge sharply if the code re-seeded).
- `reengaging_from_released_seeds_from_the_warm_start_when_a_key_matches` — warm-start entry present before the tick that drops into `Released` (see note below on exactly where the seed fires); asserts `budget_w == 99.0`.
- `calib_budget_w_stays_at_default_through_the_lut_sweep_before_any_setbudget` — drives real LUT-sweep samples; asserts `status.budget_w` never leaves its `0.0` default before the step-test hand-off.
- `calib_context_real_wiring_lets_the_step_test_actually_settle` — regression test: with the old `CalibContext::default()` stub, `fanctrl_active` was hard-wired `false` and **no real calibration could ever settle**; with a scripted active/reconciled/steady run, asserts the step test actually leaves the settle sub-phase.
- `calibration_save_preserves_the_duty_rpm_table_and_warm_start_it_does_not_own` — the SaveState bug-fix regression test described above.
- Updated two pre-existing tests (`step_test_starts_the_burner_on_entry_and_stops_it_on_skip`, `full_calibration_persists_the_lut_with_no_gains_when_the_step_test_skips`) whose assertions encoded the *old* stubbed "SetBudget is a no-op" behavior (their own comments said so, naming this task) — updated to assert the real, now-correct behavior instead (a ryzenadj call lands, `cpu_limit_w` ends up at the floor per design §3.3's "restore the floor", not `None`).

GREEN:
```
cargo test control::controller::
```
→ **88 passed; 0 failed** (79 pre-existing + 9 new; two pre-existing tests updated, not counted as "new").

```
cargo test
```
→ **573 passed; 0 failed; 2 ignored** (up from 564 on the base commit — the 9 new controller tests).

## Two real bugs found and fixed while writing tests (assertion discipline)

1. **My own test helper bug** (`rpm_view_sample` initially omitted `cpu_temp_valid`/`cpu_temp_c`): the thermal watchdog's sensor-lost trip fired after 10 samples in my 40-sample steady-window tests, dropping `AutoState` and zeroing `budget_w` — the assertion `WarmStart::lookup(...) == Some(budget_w)` failed with `None == Some(0.0)`. Root-caused via a temporary per-sample `eprintln!` trace (removed before the final commit), fixed by adding the same `cpu_temp_c: 60.0, cpu_temp_valid: true` fields every other sample-builder in this file already carries.
2. **A genuine test-design bug**, not a product bug: my first cut of `reengaging_from_released_seeds_from_the_warm_start_when_a_key_matches` inserted the warm-start entry *after* the sample that drops into `Released`, expecting the seed to happen on the *later* re-engagement sample. Tracing `handle_mode_transition`/`run_budget_and_allocate` showed the re-seed actually happens on the **same tick** `LoopMode` transitions *into* `Released` (`budget_seeded` resets to `false` there, and the seed-check re-fires within that same `on_auto_sample` call if `due` also holds) — `Freeze::Released` then holds the just-seeded value exactly, so it's what "re-engagement" reads back later. This is also exactly why the pre-existing `reengaging_from_released_reseeds_the_budget_without_a_step` test's math (`45.053`, not `45.424`) makes sense. Fixed by moving the `warm_start.insert` before the Released-entry sample and asserting on that sample directly.

Both were caught by the assertion actually running and failing on a value the code could produce — not decorative.

## Self-review

- **Completeness:** all acceptance-criteria bullets addressed; item 9 confirmed pre-covered rather than skipped.
- **Discipline:** stayed within `src/control/controller.rs`; did not touch `calib/step.rs`, `calib/runner.rs`, `fanctrl/table.rs`, `control/budget.rs`, `state.rs` even where their design comments referenced this task by name (e.g. the `CalibContext::default()` comment in `runner.rs`/`step.rs` module docs is now stale — I updated only the controller-side comments that were literally about the call site; a follow-up could tidy those module docs, but that's out of this task's file scope).
- **Quality bar not fully reached:** I did not write dedicated tests for the AC-unplug and snapped-duty-change variants of the no-reseed rule (item 4's other two cases) — the strategy-change test exercises the identical code path (the single `budget_seeded` gate doesn't distinguish which part of the key changed), so I judged one representative test sufficient given the shared mechanism, but a reviewer may reasonably want the other two spelled out explicitly.

## Validation run

```
cd .../.worktrees/epic-fw-fanctrl-loop-6ma-integration--task-fw-fanctrl-loop-438
cargo test                          # 573 passed; 0 failed; 2 ignored
cargo clippy -- -D warnings         # 4 pre-existing dead-code errors, none in
                                     # controller.rs, none in files this task
                                     # owns/touches or changed the reachability
                                     # of (verified by diffing the SAME command
                                     # against the base commit: 11 -> 4, this
                                     # task's own wiring resolved the other 7 —
                                     # Freeze::Calibrating, WarmStart, and
                                     # DutyRpmTable::refine + its 3 constants)
cargo fmt --check -- src/control/controller.rs   # clean (fopdt.rs/plant.rs
                                                  # diffs are pre-existing,
                                                  # untouched by this task)
```

## Concerns for the reviewer

- The remaining 4 `cargo clippy -D warnings` dead-code errors (`Budget::set_gains`, `fanctrl::client::resolve_curve`, `Curve::continuous_duty_at`/`duty_at`, `EcAverage::is_seeded`/`sample_count`) are pre-existing project debt in files this task doesn't own/touch — confirmed present (as part of an 11-item list) on the base commit `dbfeecd` with the identical command. Flagging rather than fixing, since fixing would mean editing files outside this task's scope.
- Item 4's AC-unplug and snapped-duty-change no-reseed cases are covered by the same code path as the strategy-change test I did write, not by their own dedicated tests (see self-review above).

## Fix round 1 (review finding: missing dedicated tests for two of the three no-reseed cases)

**Finding:** Brief step 4 mandates "one case each" for the no-reseed rule — a
strategy change, a snapped-duty change, and an AC unplug. Only the
strategy-change case had a dedicated test
(`a_strategy_change_re_keys_without_reseeding_the_budget`). This was
self-flagged in the "Quality bar not fully reached" section above as a
judgment call (one test judged sufficient given the shared `budget_seeded`
gate) that a reviewer might reasonably want spelled out explicitly. The
review confirmed it as a literal enumerated acceptance-criterion gap, not a
behavioral bug.

**Fix:** Added two more tests in `src/control/controller.rs`, same test
module, immediately after `a_strategy_change_re_keys_without_reseeding_the_budget`,
mirroring its two-parallel-sessions pattern exactly:

- `an_on_ac_change_re_keys_without_reseeding_the_budget` — `rekey`'s LAST
  tick flips `on_ac` (false -> true) instead of strategy, with `999.0`
  pre-seeded under `WarmStart::key("quiet16", 36, true)`. Asserts
  `base.status().budget_w == rekey.status().budget_w` after the flip.
- `a_snapped_duty_change_re_keys_without_reseeding_the_budget` — `rekey`
  issues `Command::SetFanTarget(3380.0)` before its LAST tick, snapping
  `target_duty` from 36 (the default 3000 RPM target) to 40 (the table's
  exact seeded point for 3380 RPM — asserted as an explicit premise via
  `duty_rpm_table.duty_for_rpm`), with `999.0` pre-seeded under
  `WarmStart::key("quiet16", 40, false)`. Asserts the same `budget_w`
  equality after that tick.

Both follow the same structure as the existing strategy-change test: three
identical warm-up ticks establishing the "premise: identical trajectory so
far" baseline, then a fourth tick where only `rekey` changes the key
component under test, pre-seeded with a decoy value (`999.0`) that would
pull `u` sharply off course if the code incorrectly reseeded on a key
change. Landing bit-identical on `budget_w` proves that tick's delta-u is
the ordinary PI increment, not a reseed — for all three key components
(strategy, on_ac, target_duty), not just strategy.

Verified `run_budget_and_allocate`'s on-AC-edge block (`on_ac_suppress_until`,
~line 1437-1444) only gates the CPU/GPU actuator write-verdict re-read
grace, not `u`/`budget_w` itself, so the on_ac test's flip tick cannot be
confounded by that mechanism. Verified `target_duty` is recomputed every
`ALLOC_PERIOD_S` tick from `status.fan_target_rpm` via
`duty_rpm_table.duty_for_rpm` (~line 1461), ahead of the same tick's arbiter
call — so `SetFanTarget` issued just before the final `on_sample` re-keys on
that same tick, matching the strategy/on_ac tests' single-tick change shape.

### Validation run

```
cd /var/home/alepar/AleCode/fw-fan-quiet/.claude/worktrees/fw-fanctrl-loop/.worktrees/epic-fw-fanctrl-loop-6ma-integration/.worktrees/epic-fw-fanctrl-loop-6ma-integration--task-fw-fanctrl-loop-438
cargo test --bin fw-fan-quiet re_keys_without_reseeding
# 3 passed; 0 failed (a_strategy_change_..., an_on_ac_change_..., a_snapped_duty_change_...)

cargo test --bin fw-fan-quiet
# test result: ok. 575 passed; 0 failed; 2 ignored; 0 measured; 0 filtered out

cargo clippy --bin fw-fan-quiet --tests
# same pre-existing dead-code warnings as base (Budget::set_gains,
# fanctrl::client::resolve_curve, Curve::continuous_duty_at/duty_at,
# EcAverage::is_seeded/sample_count) — none new, none in the added tests
```

### Commit

`b89adaf6899e54722a50c1e9b091d1dea8560304` — test(control): cover on_ac and
snapped-duty re-key variants of the no-reseed rule (task 20 review round 1)

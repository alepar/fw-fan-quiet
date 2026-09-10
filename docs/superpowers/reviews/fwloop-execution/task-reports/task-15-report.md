# Task 15 report: TUI + telemetry surface (fw-fanctrl-loop-mjv)

## Summary

On entry, the task worktree already carried a completed implementation commit
(`599cbf8`, from a prior run of this same task) sitting on the branch's
original base `f243959`, but the branch had not been rebased onto the
integration branch's later tip `cf53a2d` (which landed `fw-fanctrl-loop-24s`,
the adaptation-tier removal in `src/control/controller.rs`, in between). Per
`bd comments fw-fanctrl-loop-mjv`, the user had already left explicit,
verified conflict-resolution guidance for exactly this rebase. This run's
work was: run `bd comments`, confirm the prior implementation still matches
the brief, execute the rebase per that guidance, and verify the gate
(`cargo test` + `cargo clippy`).

The rebase was **not optional busywork** — without it the branch would not
compile against the current integration tip (it referenced
`trim_rpm`/`gain`/`model_*` `Record::Decision` fields that `fw-fanctrl-loop-24s`
had already deleted from `controller.rs`'s construction site along with the
whole adaptation tier).

## What I did

1. `bd comments fw-fanctrl-loop-mjv` — read the binding conflict-resolution
   guidance (see full text in the bead's comment thread; summarized below).
2. Verified the branch state: `task-fw-fanctrl-loop-mjv` at `599cbf8`,
   merge-base with `epic-fw-fanctrl-loop-6ma-integration` at `f243959`;
   integration tip at `cf53a2d` (contains `fw-fanctrl-loop-24s`'s adaptation-tier
   removal, landed after this task's branch was cut).
3. `git rebase epic-fw-fanctrl-loop-6ma-integration` — conflicted, exactly as
   predicted, only in `src/control/controller.rs` (single hunk, the
   `apply_effects` decision-construction closure). `src/main.rs`,
   `src/telemetry.rs`, `src/ui/view.rs` applied clean.
4. Resolved per the bd comment's exact recipe:
   - `git checkout --ours src/control/controller.rs` (during rebase, "ours" =
     the integration side being rebased onto) to take the epic's already-landed
     controller.rs verbatim (which no longer has the Kalman/trust/cooldown
     machinery, `model_snapshot` binding, or the two now-nonexistent tests
     this task's original diff had deleted — all already gone on the epic
     side, confirmed absent by the bd comment's own `git grep`).
   - Applied the one prescribed edit inside the `decision` closure in
     `apply_effects`: replaced the removed-tombstone comment plus
     `trim_rpm: None, gain: None, model_a: None, model_b: None, model_e: None,
     model_c: None,` with this task's three fields, `t_star: status.t_star_c,
     budget_w: status.budget_w, freeze: None,` — keeping the epic's flatter
     single-expression closure form.
   - `git add src/control/controller.rs` and `git rebase --continue`.
5. Verified the resulting diff of `controller.rs` against the integration tip
   (`git diff cf53a2d HEAD -- src/control/controller.rs`) is *only* that
   9-line field swap — nothing else touched in a file outside this task's
   declared `filesTouched`, matching the bd comment's "keep it minimal"
   instruction. `src/main.rs`, `src/telemetry.rs`, `src/ui/view.rs` are
   byte-identical to the pre-rebase commit (`git diff 599cbf8 HEAD -- <those
   three files>` is empty).
6. Ran the gate: `cargo build`, `cargo test` (full suite), `cargo clippy
   --all-targets`.

## Implementation (carried over from the original commit, now rebased)

- `src/ui/view.rs`: the `mode A|B|rel · T* · ma · duty -> rpm · budget`
  header segment (design §3.5), sourced from `ControlStatus`'s
  `loop_mode`/`t_star_c`/`ec_ma_c`/`duty_cmd`/`snapped_rpm`/`budget_w`.
  Rendering + severity styling for the seven `StatusFlag` variants
  (`FanctrlLost`, `EcMismatch`, `SteepCurve`, `CurveInvalid`, `GpuHot`,
  `NvmeHot`, `ReadbackBlind`) — yellow for warning, gray for info — with a
  severity-first `render_priority` so a warning flag always outranks an info
  flag when both are present (verified: `CurveInvalid` outranks
  `SteepCurve`). `CalibProgressLite.phase` renders as the plain string
  verbatim (`"lut"`, `"step"`), no enum or mapping table.
- `src/telemetry.rs`: `Record::Sample` gains
  `ec_max`/`ec_argmax`/`ec_ma`/`nvme_c`/`fanctrl_speed`/`fanctrl_active`/`strategy`
  (via the new `Record::sample` constructor, since `ec_ma` is the
  controller's stateful live boxcar average, not on `Sample` itself);
  `Record::Decision` drops `trim_rpm`/`gain`/`model_a`/`model_b`/`model_e`/`model_c`
  and gains `t_star`/`budget_w`/`freeze`. `SCHEMA_VERSION` bumped 1 -> 2.
- `src/main.rs`: the one `Record::Sample` call site updated to
  `Record::sample(s, model.status.ec_ma_c)`.
- `src/control/controller.rs` (not in `filesTouched`, unavoidable call-site
  fixup for the wire-format change per the bd comment guidance): the
  `apply_effects` decision closure's `Record::Decision` construction updated
  to `t_star: status.t_star_c, budget_w: status.budget_w, freeze: None,`.
- `src/model.rs`: no changes needed — the view reads the new fields straight
  off `model.status` (already flowing through `Event::Status`).

## Tests / results

Full suite (`cargo test`, from the rebased worktree):

```
test result: ok. 569 passed; 0 failed; 2 ignored; 0 measured; 0 filtered out; finished in 2.00s
```

(2 ignored are the pre-existing `nvml_smoke_reads_real_gpu` hardware test,
unrelated to this task.)

Focused runs covering this task's acceptance criteria:

`cargo test ui::view::` — 32 passed, 0 failed, including:
- `header_segment_for_temp_loop_mode_a`, `header_segment_for_rpm_loop_mode_b`,
  `header_segment_for_released_mode` — one snapshot per mode.
- `each_new_flag_renders_its_name`, `warning_severity_new_flags_are_yellow`,
  `info_severity_new_flags_are_gray` — per-flag snapshots.
- `curve_invalid_outranks_steep_curve` — the required ranking test
  (warning-severity flag outranks info-severity flag when both present).
- `calib_wizard_renders_lut_phase_verbatim`,
  `calib_wizard_renders_step_phase_verbatim` — `phase` rendered verbatim for
  both `"lut"` and `"step"`.

`cargo test telemetry::` — 10 passed, 0 failed, including:
- `sample_line_carries_the_new_sensor_columns` — serialised sample line
  contains every listed sample field.
- `decision_line_drops_adaptation_tier_fields_and_carries_the_new_ones` —
  asserts the new decision fields present *and* asserts the absence of
  `trim_rpm`/`gain`/`model_*` explicitly by substring on the serialised JSON
  line, not only by struct shape (matches the brief's step 4 requirement).
- `sample_line_new_columns_are_null_without_a_reading` — new optional sample
  columns null when unavailable.

`cargo clippy --all-targets`: 0 errors, 84 warnings, all pre-existing
dead-code lints in files this task did not touch (`sensors/ec.rs`,
`fanctrl/table.rs`, `types.rs`) — confirmed by grepping clippy's output for
`view.rs|telemetry.rs|model.rs|controller.rs`: zero matches. Output pristine
with respect to this task's files.

`cargo build`: clean, same warning set as clippy, no errors.

## TDD evidence

This is a rebase-and-verify run on top of an already-implemented,
already-TDD'd commit from a prior run of this task. I did not re-derive
RED/GREEN evidence (no new test-first work was done this run — the only new
code is the 3-field swap in `controller.rs`, a mechanical wire-format
call-site fixup with no independent behavior to TDD, prescribed verbatim by
the bd comment). The full green suite above (569/569, including every
acceptance-criteria test named in the brief) is the evidence that the
rebased result is correct.

## Self-review

- Diffed `controller.rs` against the integration tip: exactly the 9-line
  field swap the bd comment prescribed, nothing more.
- Diffed the other three touched files against the pre-rebase commit: byte
  identical — the rebase did not silently drop or alter anything in
  `view.rs`/`telemetry.rs`/`main.rs`.
- Confirmed `ControlStatus.t_star_c: Option<f64>` and `budget_w: f64` exist
  on the epic-side `ControlStatus` (lines 326/343 of the rebased
  `controller.rs`) and that `Record::Decision.freeze` is `Option<&'static
  str>`-typed in `telemetry.rs`, so `freeze: None` needs no coercion — no
  compiler surprises, confirmed by the clean build.
- No stray warnings introduced by this task's files.

## Files changed (this commit, relative to `cf53a2d`)

- `src/ui/view.rs`
- `src/telemetry.rs`
- `src/main.rs`
- `src/control/controller.rs` (minimal call-site fixup, not in `filesTouched`,
  per bd comment guidance)

## Concerns

None. The rebase resolution matches the bd comment's guidance exactly and
the full test suite is green with pristine output on every file this task
touched.

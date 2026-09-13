# Task 18 report: Calibration step test (fw-fanctrl-loop-0nv)

## Status: IMPLEMENTED

## What I implemented

Replaced the matrix-based calibration phase with the design's step-test
phase (design doc §3.3). `bd comments fw-fanctrl-loop-0nv` had no comments
to override the brief.

### `src/calib/step.rs` (new)

- `CalibContext { ec_ma: Option<f64>, ec_mismatch: bool, fanctrl_active:
  bool, argmax_controllable: bool, budget_bounds: (f64, f64) }`, deriving
  `Default` — the runner↔controller interface from §3.3.
- `StepTest`: a self-contained state machine (`enter`, `on_sample`),
  `Sub::Settle` → `Sub::Step`:
  - `enter(budget_bounds)` unconditionally starts the burner and holds the
    budget at the floor (`SetBudget(lo)`), **before any gate is ever
    checked** — the ordering fact the brief calls out.
  - Settle: accumulates EC-MA/RPM/CPU-draw/GPU-draw windows only when
    `fanctrl_active && !ec_mismatch && argmax_controllable` (plus
    `ec_valid`/`fan_valid`/`ec_ma.is_some()`) all hold on that sample;
    an unmet gate withholds accumulation (windows reset) rather than
    aborting — this is what lets a scripted idle-uncontrollable run
    proceed once loaded rather than self-skipping. Settled = EC MA flat
    ≤0.5 °C over 60 samples **and** RPM steady (reusing
    `steady::STEADY_N`/`STEADY_RPM_TOLERANCE`); gives up (Noted skip) at
    the 300-sample (5 min) cap.
  - On settle: baseline CPU/GPU draw is the 60-sample tail mean; steps the
    budget to `lo + 30 W` (`STEP_W`).
  - Step: hard gates (`fanctrl_active`/`ec_mismatch`) and the EC-95 °C and
    argmax-label checks now abort **immediately** (not at a cap) — once
    real data is being recorded, continuing through a discontinuity would
    poison the fit. Argmax identity is tracked via the actual
    `EcLabel` (`Sample.ec.argmax`), not just the controllability bool, so
    a handover between two controllable sensors is also caught, not only
    an uncontrollable one. `NeedsGpuLoad` nags every 10 samples while GPU
    utilization is below 90 % (mirrors `lut_sweep::PIN_UTIL_MIN_PCT`,
    restated locally since that constant is private to `lut_sweep`).
    Records `(t, ec_ma)`/`(t, rpm)` series; concludes at the 300-sample cap
    or once the EC average has been flat (same 0.5 °C band) for 90
    samples.
  - Conclusion: computes the measured total power delta (`(final_cpu −
    baseline_cpu) + (final_gpu − baseline_gpu)`, both 60-sample tail
    means) — **never `STEP_W`** — rejects an unloaded step
    (`delta_w < 1.0 W`), else calls `fit_fopdt` on each series with that
    delta and `derive_gains` (both already built by fw-fanctrl-loop-4aj);
    any rejection skips with a `Noted` reason and keeps defaults. A
    landed fit emits `Fitted { gains, fitted_at }` with `fitted_at =
    round(s.t_mono)` (the "sample clock", not `SystemTime::now()`).
  - Every terminal path (settle timeout, mid-step abort, unloaded step,
    rejected fit, or a landed fit) restores the floor (`SetBudget(lo)`)
    and stops the burner.

### `src/calib/runner.rs`

- `Phase` is now `LutSweep → StepTest → Done` (`Aborted` unchanged, for
  the user-Esc/emergency path). `MatrixPoint`/`Fitting` are gone.
- Deleted: `MATRIX_POINTS`, `CPU_IDLE_MAX_W`, `GPU_QUIET_MAX_W`,
  `GPU_BAND_FRAC`, `GPU_UTIL_ACTIVE_PCT`, `MIN_DWELL_SAMPLES`,
  `POINT_TIMEOUT_SAMPLES`, `TIMEOUT_SPREAD_MAX_RPM`, `WINDOW_CAP`,
  `enter_matrix_point`, `on_matrix_sample`, `record_matrix_point`,
  `finish`, `fail`, `tail_spread`, the `ThermalModel`/`CalibPoint` import
  and use, and every test that exercised them.
- `RunnerEffect`: removed `SetCpuW` (only the matrix phase produced it)
  and `Failed` (nothing constructs it any more — every step-test
  rejection is a graceful skip-and-finish, never a hard failure that
  discards the swept LUT); added `SetBudget(f64)` and `Noted(String)`;
  `Fitted` now carries `{ gains: LoopGains, fitted_at: u64 }` instead of
  the old thermal-model coefficients (`a, b, e, c, max_residual`).
  `BURNER_THREADS` stays (still used, now by `step.rs`).
- `on_sample(&mut self, s: &Sample, ctx: &CalibContext)`: `LutSweep`
  ignores `ctx` except on `SweepEffect::Finished`, where it calls
  `self.step.enter(ctx.budget_bounds)`. `StepTest` forwards to
  `self.step.on_sample`, watches the returned effects for `Fitted` or
  `Noted`, and on either appends `SaveState(PersistedState { lut,
  calibrated_at, loop_gains: <gains or None>, ..default })` +
  `Finished`, moving to `Phase::Done`. The swept LUT is always saved
  (skip or fit); `loop_gains` is `None` on a skip ("keep the defaults").
- `progress().phase` is `"lut"` / `"step"` / `"done"` / `"aborted"` as
  plain strings (was `"lut sweep"` / `"matrix"` / `"fitting"` / ...).
  `ui/view.rs`'s existing `calib_wizard_renders_lut_phase_verbatim` /
  `_step_phase_verbatim` tests (Task 15) already expected exactly these
  strings and needed no change.
- `abort()` is unchanged in shape (`StopBurner, ReleaseCpu, ReleaseGpu`,
  bypassing the budget system) — deliberate: an emergency/user-abort
  should not depend on the not-yet-wired `SetBudget` flow, regardless of
  which phase (LutSweep or StepTest) is running.

### `src/calib/mod.rs`

Added `pub mod step;` (one line).

### `src/control/controller.rs` — call site + necessary follow-on

The brief scoped this file to "a one-line call-site change only" for the
*production* path, and that line is exactly one edit:

```rust
Some(runner) => runner.on_sample(s, &CalibContext::default()),
```

with a comment naming `fw-fanctrl-loop-438` (the task that wires the real
signals through). Two more mechanical changes were unavoidable
consequences of the `RunnerEffect` shape this task owns, not scope
creep — the file would not compile otherwise:

- `apply_calib_effects`: `SetCpuW`'s match arm is deleted (the variant no
  longer exists); `SetBudget(_) => {}` is added (ignored, with the
  `fw-fanctrl-loop-438` comment); `Fitted`'s arm now destructures `{
  gains, fitted_at }`; a new `Noted(reason)` arm logs and raises
  `"calib:skipped"` (renamed from `"calib:failed"`, since nothing calls it
  a failure any more — see `rank()`); the `Failed` arm (and its
  `ended = true`) is removed.
- The test module's calibration-integration block (`use
  crate::calib::runner::MATRIX_POINTS`, `matrix_point_sample`,
  `drive_matrix_point`) referenced deleted symbols and could not compile;
  I rewrote it against the step test:
  - `sweep_pinned`/`drive_sweep` kept, updated for `"lut"`/`"step"`.
  - New `settle_sample()` + `drive_step_test_to_skip()`: since the call
    site always passes `CalibContext::default()` this task,
    `fanctrl_active` is always `false`, so any drive through the
    controller times out at the 5-minute settle cap and skips — that is
    exactly what these tests exercise (burner start/stop bookkeeping
    around a step test the controller cannot yet *complete*).
  - `calibration_effects_drive_actuators_and_burner` →
    `step_test_starts_the_burner_on_entry_and_stops_it_on_skip`: burner
    starts as soon as the sweep hands off (before any gate), no
    `ryzenadj` calls happen (SetBudget is ignored), burner stops once the
    settle cap skips.
  - `repeated_needs_load_nags_are_noted_for_telemetry` →
    `repeated_sweep_needs_load_nags_are_noted_for_telemetry`: the
    original test's *matrix point 4* GPU-idle nag is gone (no matrix), so
    this now drives the LUT sweep's own idle-GPU nag instead — same
    StatusChanged→Noted assertion shape, same intent (nag noted for
    telemetry), different phase.
  - `abort_calibration_releases_and_returns_to_monitor`,
    `quit_during_calibration_aborts_then_restores`,
    `emergency_during_calibration_aborts_it`: now abort/quit/emergency
    mid-*settle* (10 `settle_sample()`s with the burner running) instead
    of mid-matrix-point; same assertions otherwise.
  - `full_calibration_persists_state_and_keeps_lut` →
    `full_calibration_persists_the_lut_with_no_gains_when_the_step_test_skips`:
    drives sweep + the 300-sample settle-cap skip, asserts the LUT is
    still persisted and `loop_gains == None` (added assertion — the
    "keep defaults" behavior wasn't observable at all in the old matrix
    flow's controller-level test).

## The `fitted_at` scope question (recorded per the "read the design doc,
say where it disagrees" constraint)

§2.4's persisted `LoopGains` lists `tau_s, theta_s, k_c_per_w,
k_rpm_per_w, fitted_at` alongside the four PI-gain fields. The `LoopGains`
that actually exists (`control::budget::LoopGains`, fw-fanctrl-loop-834)
has **only** the four PI-gain fields — `fopdt.rs`'s own module doc
(written by fw-fanctrl-loop-4aj) documents this deviation explicitly and
says stamping `fitted_at` is "explicitly owned by the calibration runner
(fw-fanctrl-loop-0nv)". But this task's `filesTouched` is `runner.rs`,
`step.rs`, `mod.rs`, and `controller.rs` (call site only) — it does **not**
include `budget.rs` or `state.rs`, so I cannot extend `LoopGains` or
`PersistedState` with a `fitted_at` field without touching files outside
this task's declared scope (and `state.rs`'s own round-trip test
(`new_schema_round_trips_populated_warm_start_and_gains`) asserts the
current 4-field shape byte-for-byte).

Resolution: `fitted_at` is carried on `RunnerEffect::Fitted { gains,
fitted_at }` — fully within `runner.rs`'s own `RunnerEffect` enum, which
this task owns outright — and asserted on directly in
`step::tests::successful_fit_stamps_fitted_at_from_the_sample_clock` and
`runner::tests::full_happy_path_walks_sweep_burner_settle_step_and_saves_gains_with_fitted_at`.
It is **not** persisted to `PersistedState`/`state.json` in this
increment; `PersistedState.loop_gains` carries the 4-field `LoopGains`
only. Task 20 (fw-fanctrl-loop-438, "Controller hooks: warm-start,
refinement, calibration") is the one that wires `SetBudget` for real, and
its own `filesTouched` is `controller.rs` only — so persisting
`fitted_at` durably is a real gap this epic doesn't close anywhere I can
see; I'm flagging it here rather than silently working around it. It may
need its own follow-on bead touching `budget.rs`/`state.rs` together.

## The "measured per-axis power delta" reading

The design doc/bead both say "measured per-axis power delta — never
`STEP_W`". `Fopdt`/`derive_gains` operate on one scalar `step_w` per
signal fit (EC average, RPM), and `LoopGains`'s gains are W-per-°C /
W-per-RPM against the **total** budget `u`, not per-axis — so I read
"per-axis" as "the delta measured on each axis (CPU, GPU), summed to the
total applied delta" (`(final_cpu − baseline_cpu) + (final_gpu −
baseline_gpu)`), used uniformly as `step_w` for both fits. This is
exercised directly by
`step::tests::gain_is_derived_from_the_measured_delta_not_the_nominal_thirty_watts`,
which drives a step whose actual measured delta (24 W, split unevenly 8/16
between CPU/GPU) differs from the nominal 30 W and asserts the derived
`Kc` matches the ground-truth-K derivation rather than a nominal-30W
variant.

## ASSERTION DISCIPLINE notes

- `settle_gives_up_at_the_five_minute_cap_and_keeps_defaults` and
  `full_calibration_persists_the_lut_with_no_gains_when_the_step_test_skips`
  assert `loop_gains == None` / absence of `Fitted` — a scripted context
  where `fanctrl_active` never clears makes the "settle never completes"
  branch the only reachable one, so these are not decoration: a bug that
  let settle proceed anyway (e.g. an inverted gate check) would produce a
  `Fitted` effect and fail these.
- `gain_is_derived_from_the_measured_delta_not_the_nominal_thirty_watts`
  asserts the derived `kc_w_per_c` differs from a *hypothetical*
  nominal-30W-derived value by construction (computed inline as `* 0.8`,
  the ratio a K-based-on-30W-instead-of-24W would produce) — a
  regression that swapped `delta_w` for `STEP_W` in the `fit_fopdt` call
  would make this assertion fail (I confirmed by temporarily hard-coding
  `STEP_W` into `conclude_step`'s fit calls locally and re-running the
  test — it failed as expected — then reverted).
- `argmax_label_change_mid_step_skips_and_keeps_defaults` builds two
  *different* `EcLabel`s via two separate hwmon fixture directories
  (`apu@4c` then `cpu@4c`) — both controllable, so this specifically
  exercises the label-identity check, not the controllability bool (a
  bug that only compared `argmax_controllable` would pass this test only
  by accident; I did not additionally test that narrower case since the
  brief only asks for "the argmax label changes").
- I did not write a dedicated test asserting the exact numeric value of
  `MIN_STEP_DELTA_W` (1.0 W) as a boundary — `unloaded_step_skips_...`
  uses `delta_w = 0.0` (well clear of the boundary either way), so the
  exact threshold constant is unmeasured, not asserted; noting this as
  unmeasured rather than claiming coverage I don't have.

## Test results

- `cargo test --bin fw-fan-quiet` (full suite): **605 passed, 0 failed,
  2 ignored** (same 2 pre-existing ignores as baseline, unrelated to this
  task).
- `cargo test --bin fw-fan-quiet calib::`: 60/60 passing (step.rs: 14
  tests; runner.rs: 9 tests; unchanged fopdt/lut_sweep/steady/burner
  suites: 37 tests).
- `cargo test --bin fw-fan-quiet control::controller::`: 61/61 passing.
- `cargo clippy --all-targets -- -D warnings`: **not clean crate-wide**,
  but this is a pre-existing baseline condition, not something this task
  introduced or is positioned to fix. I verified this precisely: I
  captured the clippy error list on the unmodified base commit
  (`fbe1f14`, via `git stash`) — **81 errors**, entirely dead-code
  findings in `mode.rs` (the `Arbiter`, unwired), `thermal_model.rs`,
  `trust.rs`, `curve.rs`, `table.rs`, `sensors/ec.rs`'s `EcAverage`/
  `is_controllable`, and `types.rs`'s `fanctrl_freshness` field — none of
  it touched by this task, all of it either destined for Task 21's
  deletion sweep or awaiting a wiring task this epic hasn't reached yet.
  With this task's changes, the same command reports **70 errors**: a
  net **improvement of 11** (this task's own use of `fit_fopdt`,
  `derive_gains`, `Fopdt` and the FOPDT constants in `step.rs` makes
  `fopdt.rs` — previously entirely dead, since the old matrix runner
  never called it — used for the first time), and the *only* two
  genuinely new findings were style lints in my own `step.rs` test code
  (`manual_contains`, `expect_fun_call`), which I fixed. I diffed the
  full before/after error-name lists twice (`diff` on sorted
  `grep -E '^error'` output) to confirm no other new category appeared.
  I did not touch `mode.rs`/`thermal_model.rs`/`trust.rs`/`curve.rs`/
  `table.rs`/`sensors/ec.rs`/`types.rs` — none are in this task's
  `filesTouched`, and "fixing" their dead-code status by, say, wiring in
  the `Arbiter` would be exactly the scope-widening the Global
  Constraints forbid.
- `rustfmt --edition 2024 --check` on each of the 4 touched files
  individually: clean (0 diffs) for `step.rs`, `runner.rs`,
  `controller.rs`; `mod.rs`'s diff is the one intended line.
  **Caveat for future sessions in this repo:** running `cargo fmt` (or
  plain `rustfmt` given more than one file, or a file that itself
  declares `mod`) reformats the *entire* module tree reachable from any
  root you pass it — it silently rewrote `fopdt.rs`, `budget.rs` and
  `plant.rs` (all outside this task's scope) the first two times I tried
  it. I caught this via `git status` before committing and reverted those
  three files (`git checkout --`) each time; the final commit touches
  only the 4 intended files. Format one bare leaf file at a time (never
  `mod.rs`, never more than one file together) if you need `rustfmt` in
  this crate.
- Verified `grep -rn "fit_batch\|MATRIX_POINTS\|MatrixPoint\b" src/calib/`
  and `grep -rn "record_matrix_point\|Phase::Fitting\|::Fitting\b"
  src/calib/ src/control/controller.rs` both return zero hits.

## Files changed

- `src/calib/mod.rs`
- `src/calib/runner.rs`
- `src/calib/step.rs` (new)
- `src/control/controller.rs`

## Self-review / concerns

- The controller-level tests can only exercise the *skip* path (settle
  never completes, since `CalibContext::default()` always has
  `fanctrl_active: false`) — the happy path (a landed fit) is exercised
  end-to-end only in `runner.rs`'s own tests, which drive `CalibRunner`
  directly with scripted `CalibContext`s. That's the correct boundary
  given this task's scope (task 20 owns wiring the controller's real
  context), but it means "the controller successfully completes a step
  test with a real fit" has no test coverage anywhere yet — worth keeping
  in mind when task 20 lands.
- `StepTest::stepping()`/`needs_load()` are small accessor methods added
  for `runner.rs`'s `progress()` to consume; they're plain reads, no
  behavior of their own.
- I did not add a test pinning `MIN_STEP_DELTA_W`'s exact value (1.0 W)
  at the boundary — see the assertion-discipline note above.

## Head

`539c8165b4bc5f98ea116e19d168bb2d37e6b6da` on branch
`task-fw-fanctrl-loop-0nv`, working tree clean (`git status --short`
empty).

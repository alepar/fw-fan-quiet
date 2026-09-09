## Task 18: Calibration step test

**Bead:** `fw-fanctrl-loop-0nv`

**filesTouched:** `src/calib/runner.rs`, `src/calib/step.rs`, `src/calib/mod.rs`,
`src/control/controller.rs`

`src/control/controller.rs` — **a one-line call-site change only**: pass
`CalibContext::default()` and ignore `SetBudget` until Task 20 wires them.

### Global constraints

All of "Global Constraints" above applies. Normative: **§3.3**.

### The ordering fact that makes this task correct

**The burner starts first, before the settle detection.** Two-stage gating: `fanctrl_active` and
`!ec_mismatch` are checked **before** the settle; **`argmax_controllable` is checked only after
the burner is running**. At the floors, this machine's own idle fixture has `ambient` above
`apu` — an up-front controllability check would self-skip **every** run. This is the single
most important sequencing constraint in the task.

### What this task owns

Runner phases `LutSweep -> StepTest -> Done`. **Deleted:** `MATRIX_POINTS`, `MatrixPoint`,
`Fitting`, `record_matrix_point`, the `fit_batch` use and their tests.
`on_sample(&Sample, &CalibContext)` where `CalibContext: Default`.

New `src/calib/step.rs`: burner started first; then settle detection (EC MA flat within 0.5 °C
over 60 s **and** RPM steady, cap 5 min); then a `budget_bounds.0 + 30 W` step requested via the
new `RunnerEffect::SetBudget(w)`; `NeedsLoad` nagged while GPU utilisation is below the pin
threshold; a 5 min record of EC MA, RPM and **measured** applied power per axis; then
`fit_fopdt` + `derive_gains` on the **measured per-axis power delta — never the nominal 30 W**;
stamping `fitted_at` from the sample clock.

**Skip with a `Noted` reason** when either gate fails, when the applied power never rose, when
the EC max exceeds 95 °C (abort **and restore the floors**), when the argmax label changes
mid-step, or when the fit is rejected.

`RunnerEffect::SaveState` now carries `loop_gains`. `progress().phase` reports `"lut"` / `"step"`
as **plain strings**.

### Acceptance criteria (verbatim from the bead)

> runner end-to-end test on the fake seams walks sweep -> burner -> settle -> step -> `SaveState`
> with gains carrying `fitted_at`, driven by scripted `CalibContext`s; the burner starts before
> the settle hold and stops after; a scripted run whose idle argmax is uncontrollable but whose
> loaded argmax is controllable **proceeds** rather than self-skipping; an unloaded step, a 95 °C
> abort, a mid-step argmax change and a rejected fit each skip with a `Noted` reason and keep
> defaults; the gain is computed from the measured per-axis delta, not 30 W; no
> `fit_batch`/`MATRIX_POINTS` symbol remains in `calib/`.

### Implementation steps (TDD)

1. **Test first:** a runner end-to-end walk on the fake seams — sweep, burner, settle, step,
   `SaveState` — with the emitted gains carrying `fitted_at`, driven by scripted `CalibContext`
   values. Then build the phase machine and `CalibContext`.
2. **Test first — the ordering test:** the burner **starts before** the settle hold begins and
   **stops after** the step completes, asserted against the effect order. Then implement.
3. **Test first — the self-skip regression:** a scripted run whose **idle** argmax is
   uncontrollable but whose **loaded** argmax is controllable **proceeds**. Then implement the
   two-stage gating with `argmax_controllable` evaluated only after the burner runs.
4. **Test first:** settle detection needs EC MA flat within 0.5 °C over 60 s **and** RPM steady,
   and gives up at the 5 min cap. Then implement.
5. **Test first:** the step requests `budget_bounds.0 + 30 W` via `RunnerEffect::SetBudget(w)`,
   and `NeedsLoad` is nagged while GPU utilisation is below the pin threshold. Then implement.
6. **Test first — the measured-delta test:** with a scripted run where the **applied** per-axis
   power delta differs from the nominal 30 W, the derived gain is computed from the **measured**
   delta. Assert the derived value against the measured delta and assert it is **not** the 30 W
   value. Then implement.
7. **Test first, one per skip path:** an unloaded step (applied power never rose); a 95 °C abort
   (which also **restores the floors**); a mid-step argmax label change; a rejected fit. Each
   emits a `Noted` reason and **keeps the defaults**. Then implement each.
8. **Test first:** `progress().phase` is the plain string `"lut"` then `"step"`.
9. Delete `MATRIX_POINTS`, `MatrixPoint`, `Fitting`, `record_matrix_point`, the `fit_batch` use
   and their tests. **Verify:** search `src/calib/` for `fit_batch` and `MATRIX_POINTS` — zero
   hits; record it in your report.
10. Change the controller call site to pass `CalibContext::default()` and ignore `SetBudget`,
    with a comment naming `fw-fanctrl-loop-438`.
11. Run the test suite and the linter; both clean.

### Deliverable

A matrix-free calibration runner with a step-test phase whose gain derivation is measured, and
whose gating cannot self-skip on this machine's idle thermals.

---


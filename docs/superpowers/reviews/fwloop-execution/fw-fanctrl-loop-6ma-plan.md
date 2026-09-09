# Plan — fw-fanctrl-loop-6ma: Close the loop on fw-fanctrl (no learned thermal model)

Epic: `fw-fanctrl-loop-6ma`.
Design doc (the single source of truth every task reads):
`docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md`.
Reviews: `docs/superpowers/reviews/` (2 coverage rounds + 3 super-roast iterations; all
dispositions in `fwloop-coverage-ledger.md`).

Task sections are headed by a **sequential integer ordinal**. The bead id is the durable
identity for every `bd` command; the ordinal-to-bead-id mapping is the table below.

## Global Constraints (apply to every task section)

- **Goal:** auto mode holds the fans at the user RPM target by capping CPU and GPU power
  through one integrator regulating the temperature fw-fanctrl reacts to (Mode A), falling back
  to measured-RPM regulation (Mode B), with **no learned power→RPM thermal model in the
  binary**. `thermal_model.rs`, `kalman.rs`, `trust.rs`, `cooldown.rs`, `trim.rs`, the
  calibration matrix phase and the controller adaptation tier are deleted.
- **fw-fanctrl owns the fans.** This binary never writes fan speed. The socket client may send
  only `print speed` and `print all` — read-only by construction. Any test that observes a
  command log asserts nothing else was ever sent.
- **Read the design doc section your task names before writing code.** The bead text is a
  summary; the spec section is normative. Where the two disagree, the spec wins, and you say so
  in your report.
- **TDD.** Write the failing test first, then the implementation. Every task's acceptance
  criteria below are the test list; they are carried verbatim from the bead.
- **Quality gate for every task:** `cargo test` green and `cargo clippy -D warnings` clean for
  the code you touched. Do not leave a task with a `#[allow]` added to silence a warning you
  introduced.
- **Do not widen scope.** Each bead states what it **owns** and what it **consumes**. Do not
  implement a consumed symbol yourself — it exists on the integration branch by the time your
  task runs. If it does not, that is a BLOCKED condition; report it rather than stubbing it.
- **Determinism.** All simulation and plant tests use a seeded, hand-rolled RNG. No new crate
  dependency is added anywhere in this epic without saying so explicitly in the report.

### Hot files — read this before declaring your own writes

Three files are structurally hot in this epic and are named in many tasks' `filesTouched`:

- `src/control/mod.rs` — module barrel. Tasks 3, 5, 6, 11, 16, 21, 22 each add or remove **one
  `pub mod` line**. It cannot be assigned to a single task, because each new module must be
  declared by the task that creates it or the crate does not compile. Keep the edit to the one
  line you need; never reformat or reorder the file.
- `src/control/controller.rs` — the controller. Tasks 4, 6, 9, 12, 13, 18, 19, 20 touch it, but
  they are almost entirely **sequenced by dependency**, not parallel. Tasks 4/6/9/13/18 are
  restricted to the *named call sites only*; tasks 12, 19 and 20 own the file's body in turn.
  Respect the "call site only" restriction literally — an incidental cleanup elsewhere in this
  file is the most expensive merge conflict available in this epic.
- `src/main.rs` — module barrel (`mod` declarations only). Tasks 1, 2, 14, 24.

## Mapping

| N | Bead | Task name | filesTouched |
|---|------|-----------|--------------|
| 1 | fw-fanctrl-loop-9dv | Curve model + DutyRpmTable | `src/fanctrl/mod.rs`, `src/fanctrl/curve.rs`, `src/fanctrl/table.rs`, `src/main.rs` |
| 2 | fw-fanctrl-loop-blm | Capture machine fixtures + test-support layout | `tests/fixtures/fanctrl/print_all_quiet16.json`, `tests/fixtures/fanctrl/print_all_cool16.json`, `tests/fixtures/fanctrl/print_all_load.json`, `tests/fixtures/fanctrl/print_speed.json`, `tests/fixtures/hwmon/cros_ec_idle/*`, `tests/fixtures/hwmon/cros_ec_load/*`, `tests/fixtures/hwmon/cros_ec_dgpu_on/*`, `tests/fixtures/hwmon/nvme/*`, `tests/fixtures/power_supply/ACAD/online`, `tests/fixtures/ryzenadj_info.txt`, `tests/fixtures/state_v1.json`, `src/test_support/mod.rs`, `src/test_support/fixtures.rs`, `src/test_support/fakes.rs`, `src/test_support/plant.rs`, `src/main.rs` |
| 3 | fw-fanctrl-loop-834 | Budget integrator | `src/control/budget.rs`, `src/control/mod.rs` |
| 4 | fw-fanctrl-loop-fo1 | Controller status surface | `src/control/controller.rs`, `src/ui/view.rs`, `src/telemetry.rs` |
| 5 | fw-fanctrl-loop-mm2 | Guards (dGPU, NVMe) + config keys | `src/control/guards.rs`, `src/config.rs`, `src/control/mod.rs` |
| 6 | fw-fanctrl-loop-zct | Allocator: scalar budget split | `src/control/allocator.rs`, `src/control/mod.rs`, `src/control/controller.rs` |
| 7 | fw-fanctrl-loop-58u | fw-fanctrl socket client | `src/fanctrl/client.rs`, `src/fanctrl/mod.rs`, `src/config.rs`, `src/test_support/fakes.rs` |
| 8 | fw-fanctrl-loop-52c | EC replica + NVMe + AC sensors | `src/sensors/ec.rs`, `src/sensors/hwmon.rs`, `src/sensors/mod.rs` |
| 9 | fw-fanctrl-loop-jpg | Actuator read-back | `src/actuators/cpu.rs`, `src/actuators/gpu.rs`, `src/control/controller.rs` |
| 10 | fw-fanctrl-loop-4aj | FOPDT fit + IMC gain derivation | `src/calib/fopdt.rs`, `src/calib/mod.rs` |
| 11 | fw-fanctrl-loop-9it | Spike: settle the anti-windup rule | `src/control/spike_antiwindup.rs`, `src/control/mod.rs`, `docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md` |
| 12 | fw-fanctrl-loop-24s | Remove the adaptation tier | `src/control/controller.rs` |
| 13 | fw-fanctrl-loop-dsh | Persisted state migration | `src/state.rs`, `src/control/controller.rs` |
| 14 | fw-fanctrl-loop-sov | Sample plumbing + socket poller | `src/types.rs`, `src/sensors/sampler.rs`, `src/sensors/mod.rs`, `src/main.rs` |
| 15 | fw-fanctrl-loop-mjv | TUI + telemetry surface | `src/ui/view.rs`, `src/telemetry.rs`, `src/model.rs` |
| 16 | fw-fanctrl-loop-iym | Mode arbiter, reconciliation, feasibility | `src/control/mode.rs`, `src/control/mod.rs` |
| 17 | fw-fanctrl-loop-51b | fw-fanctrl emulator + chained plant | `src/test_support/plant.rs` |
| 18 | fw-fanctrl-loop-0nv | Calibration step test | `src/calib/runner.rs`, `src/calib/step.rs`, `src/calib/mod.rs`, `src/control/controller.rs` |
| 19 | fw-fanctrl-loop-j6s | Controller loop integration | `src/control/controller.rs` |
| 20 | fw-fanctrl-loop-438 | Controller hooks: warm-start, refinement, calibration | `src/control/controller.rs` |
| 21 | fw-fanctrl-loop-eyi | Deletion sweep | `src/control/thermal_model.rs`, `src/control/kalman.rs`, `src/control/trust.rs`, `src/control/cooldown.rs`, `src/control/trim.rs`, `src/control/mod.rs`, `src/control/gpu_pid.rs`, `TODO.md` |
| 22 | fw-fanctrl-loop-cm7 | Closed-loop acceptance + configuration smoke | `src/control/sim_tests.rs`, `src/control/mod.rs` |
| 23 | fw-fanctrl-loop-7ij | README + docs | `README.md`, `docs/research/03-control.md`, `docs/superpowers/specs/INDEX.md` |
| 24 | fw-fanctrl-loop-nsc | Integration sweep: fw-fanctrl closed loop | `src/integration_tests.rs`, `src/main.rs`, `docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md` |
| 25 | fw-fanctrl-loop-nez | Guard the infinite tread endpoint (curve/arbiter seam) | `src/fanctrl/curve.rs`, `src/control/mode.rs`, `docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md` |

### Legacy `fwloop.N` cross-reference

Bead descriptions cite the design doc's original `fwloop.N` slugs. This is the mapping; use it
when a bead says "blocked-by fwloop.X":

`fwloop.1`=9dv · `fwloop.2`=58u · `fwloop.3`=52c · `fwloop.4`=834 · `fwloop.5`=zct ·
`fwloop.6`=mm2 · `fwloop.7`=jpg · `fwloop.8`=fo1 · `fwloop.9`=sov · `fwloop.10`=iym ·
`fwloop.11`=dsh · `fwloop.12`=j6s · `fwloop.13`=0nv · `fwloop.14`=eyi · `fwloop.15`=mjv ·
`fwloop.16`=51b · `fwloop.17`=cm7 · `fwloop.18`=7ij · `fwloop.19`=438 · `fwloop.20`=blm ·
`fwloop.21`=4aj · `fwloop.22`=24s · `fwloop.23`=nsc · `fwloop.24`=9it

---

## Task 1: Curve model + DutyRpmTable

**Bead:** `fw-fanctrl-loop-9dv`

**filesTouched:** `src/fanctrl/mod.rs`, `src/fanctrl/curve.rs`, `src/fanctrl/table.rs`,
`src/main.rs`

`src/main.rs` is a barrel: add exactly one line, `mod fanctrl;`, next to the existing `mod`
declarations. Nothing else in that file.

### Global constraints

All of "Global Constraints" above applies. Normative spec sections: §2.1 (curve model), §2.3
(`DutyRpmTable`), and §Facts for the live curve point lists and the seeded table.

This task is **dependency-free**: it consumes nothing from other tasks. Unit tests use the
§Facts point lists **inline** — do **not** reach for `tests/fixtures/` (that is Task 2's
corpus, and this task must not depend on it).

### Reference data (from §Facts / §2.3, use verbatim)

- `quiet16` = (0,15) (55,15) (65,21) (75,31) (82,37) (88,55) (95,100), `movingAverageInterval` 60
- `cool16` = (0,20) (50,20) (60,30) (70,42) (85,100), interval 60
- Verified truncation case on cool16: `T_eff` 51.8 → duty 21
- Seeded `DutyRpmTable`: 15→1195, 20→1670, 27→2300, 30→2560, 36→3030, 40→3380, 44→3670,
  48→3950, 52→4180, 85→5920

### Acceptance criteria (verbatim from the bead)

> on the `quiet16`/`cool16` points — treads, T*, slopes, `nearest_tread` for a skipped integer
> resolves to the nearest lower tread and to `None` below the floor; a curve with a descending
> segment is rejected; `default()` equals the seed and a JSON without the field deserialises to
> it; interpolation, snap ties; a refinement that would invert two adjacent duties is clamped
> and the table stays monotone after 100 noisy refinements; a > 25 % jump is rejected.

### API this task owns

`Curve::from_points(Vec<(f64, u8)>) -> Result<Curve>` (file order, **rejecting any descending
segment**), `duty_at`, `tread`, `t_star`, `slope_at`, `nearest_tread(d) -> Option<u8>`,
`min_tread_duty()`. `DutyRpmTable` with `Default` = the ten seeded points (also the serde
default), `duty_for_rpm` (ties down), `rpm_for_duty`, `refine(duty, mean_rpm)`.

The non-monotone rejection surfaces **as `CURVE INVALID` at warning severity** via Task 16's
`curve_valid` input — **never** as `SteepCurve`. This task only produces the `Err`; do not add
any flag here.

### Implementation steps (TDD)

1. Create `src/fanctrl/mod.rs` with `pub mod curve; pub mod table;` and add `mod fanctrl;` to
   `src/main.rs`. Confirm the build still passes with the two files empty.
2. **Test first:** `curve::from_points` on the `quiet16` list yields a curve whose `duty_at`
   interpolates as fw-fanctrl does, and on the `cool16` list `int(duty_at(51.8)) == 21`
   (truncation, not rounding). Then implement `Curve::from_points` + `duty_at`.
3. **Test first:** `from_points` on a list with a descending duty segment returns `Err`. Then
   implement the rejection. Assert the error is a plain domain error — no flag, no panic.
4. **Test first:** `tread(d)` for each duty present in `quiet16` and `cool16`; `t_star` for a
   given `target_duty`; `slope_at(t)` in %/°C at points inside and at the boundary of each
   segment. Then implement.
5. **Test first:** `nearest_tread` for an integer duty the curve skips resolves to the nearest
   duty with a tread, **preferring lower**; below the lowest tread it is `None`.
   `min_tread_duty()` returns the lowest duty that has a tread. Then implement.
6. **Test first:** `DutyRpmTable::default()` equals exactly the ten seeded points; a JSON object
   with no `duty_rpm_table` key deserialises to that default (`#[serde(default)]`). Then
   implement the struct + serde.
7. **Test first:** `rpm_for_duty` interpolates linearly between entries and clamps flat outside;
   `duty_for_rpm` picks the nearest tread by interpolated RPM and **ties go down**. Then
   implement.
8. **Test first:** `refine(duty, mean)` applies `rpm <- 0.8*rpm + 0.2*mean`; a `mean` more than
   25 % from the current value is rejected outright; a refinement that would make the table
   non-increasing in duty is clamped so strict monotonicity holds. Then implement.
9. **Test first (property):** starting from the default, apply 100 refinements with seeded
   noise; assert the table is strictly increasing in duty after every one.
10. Run the test suite and the linter; both clean.

### Deliverable

`src/fanctrl/{mod,curve,table}.rs` compile and are unit-tested standalone. No other module
imports them yet.

---

## Task 2: Capture machine fixtures + test-support layout

**Bead:** `fw-fanctrl-loop-blm`

**filesTouched:** `tests/fixtures/fanctrl/print_all_quiet16.json`,
`tests/fixtures/fanctrl/print_all_cool16.json`, `tests/fixtures/fanctrl/print_all_load.json`,
`tests/fixtures/fanctrl/print_speed.json`, `tests/fixtures/hwmon/cros_ec_idle/*`,
`tests/fixtures/hwmon/cros_ec_load/*`, `tests/fixtures/hwmon/cros_ec_dgpu_on/*`,
`tests/fixtures/hwmon/nvme/*`, `tests/fixtures/power_supply/ACAD/online`,
`tests/fixtures/ryzenadj_info.txt`, `tests/fixtures/state_v1.json`,
`src/test_support/mod.rs`, `src/test_support/fixtures.rs`, `src/test_support/fakes.rs`,
`src/test_support/plant.rs`, `src/main.rs`

`src/test_support/fakes.rs` and `src/test_support/plant.rs` are created here **as empty
`cfg(test)` stubs only** so `mod.rs` compiles — their contents belong to Tasks 7 and 17. Do not
write fake or plant logic in this task.

`src/main.rs` is a barrel: add exactly `#[cfg(test)] mod test_support;`. Nothing else.

### Global constraints

All of "Global Constraints" above applies. Normative: §Facts (every number below is quoted from
it), §2.2, §2.9.

**These are capture fixtures, not synthesised ones.** Where a live capture is possible on this
machine, capture it. Where it is not, reproduce the recorded values from §Facts **exactly** and
say in your report which files were synthesised from §Facts rather than captured. Do not invent
a value §Facts does not state.

### Fixture contents required

- `fanctrl/print_all_quiet16.json`, `fanctrl/print_all_cool16.json` — verbatim live `print all`
  replies with each strategy resolved (`strategy`, `speed`, `temperature`,
  `movingAverageTemperature`, `effectiveTemperature`, `active`, full config incl. `strategies`).
  Their curve points must be exactly the §Facts lists, `movingAverageInterval` 60.
- `fanctrl/print_speed.json` — a `print speed` reply.
- `hwmon/cros_ec_idle/` — `name` plus `temp*_label`/`temp*_input`: ambient 47850, charger 44850,
  apu 43850, cpu@4c 40850, three gpu -150, and **`gpu_temp@40` as a label with no `_input`
  file** (the ENODATA convention this task owns).
- `hwmon/cros_ec_load/` — cpu@4c 74850, ambient 69850, apu 69850, charger 63850 — paired with
  `fanctrl/print_all_load.json` (`temperature: 75.0`) captured in the same second.
- `hwmon/cros_ec_dgpu_on/` — **dGPU powered at 18.9 W and the `gpu_*` sensors still -150 /
  absent.** This fixture pins the measured fact that they never report; it replaces the round-2
  assumption that they come alive.
- `hwmon/nvme/` — a `Composite` temperature tree.
- `power_supply/ACAD/online`.
- `ryzenadj_info.txt` — captured with `ryzen_smu` **unloaded**, via the sudo pattern; must
  contain the `PPT LIMIT SLOW`, `PPT LIMIT FAST` and `STAPM LIMIT` rows Task 9 parses.
- `state_v1.json` — today's `state.json`, i.e. containing `model` / `adapt_*`, so Task 13 can
  prove the migration.

### Acceptance criteria (verbatim from the bead)

> every file above exists and is committed; `fixtures::path` resolves each; the recorded cool16
> curve reproduces `int(duty_at(51.8)) == 21` and a tread above 70 °C with slope > 2 %/°C; the
> loaded tree's rounded max (75) equals its paired `print all` `temperature`; the dGPU-on tree
> contains no positive `gpu_*` reading.

Note: the `int(duty_at(51.8)) == 21` and slope assertions are **fixture-shape** assertions.
Task 1 owns `Curve`; if it is not on your branch yet, assert the equivalent directly on the
parsed points (the cool16 segment containing 51.8 yields 21 after truncation; the segment above
70 °C rises more than 2 % duty per °C) rather than importing `Curve`.

### Implementation steps (TDD)

1. Create `src/test_support/mod.rs` declaring `pub mod fixtures; pub mod fakes; pub mod plant;`
   under `cfg(test)`, plus empty `fakes.rs`/`plant.rs`. Add `#[cfg(test)] mod test_support;` to
   `src/main.rs`. Confirm the test build compiles.
2. **Test first:** `fixtures::path("fanctrl/print_all_quiet16.json")` returns an existing path,
   resolved from `CARGO_MANIFEST_DIR` (never a relative path — tests run from varying cwd).
   Then implement `fixtures::path`.
3. Capture or reconstruct each fixture file listed above. Commit them.
4. **Test first, one per acceptance bullet:** every listed file exists and `fixtures::path`
   resolves it; the cool16 points truncate 51.8 to 21; a cool16 tread above 70 °C has slope
   above 2 %/°C; the max over positive `cros_ec_load` readings rounds to 75 and equals
   `print_all_load.json`'s `temperature`; `cros_ec_dgpu_on` contains no positive `gpu_*` value.
5. **Test first:** the ENODATA convention — `cros_ec_idle` has a `temp*_label` naming
   `gpu_temp@40` with **no** matching `_input` file, and the test asserts that absence
   explicitly (this is the convention Task 8 keys off).
6. Run the test suite and the linter; both clean.

### Deliverable

A committed fixture corpus plus `fixtures::path`, both usable by any later task without further
capture work.

---

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

## Task 4: Controller status surface

**Bead:** `fw-fanctrl-loop-fo1`

**filesTouched:** `src/control/controller.rs`, `src/ui/view.rs`, `src/telemetry.rs`

`src/ui/view.rs` and `src/telemetry.rs` edits are **compile-only** here: update call sites just
enough that the crate builds. The real rendering and serialisation are Task 15.

### Global constraints

All of "Global Constraints" above applies. Normative: §2.5 (mode names), §2.7, §2.8, §2.9,
§3.2, §3.5.

**Scope fence, stated in the bead and repeated here because it is the likeliest overreach:**

> The repo-wide symbol sweep is **not** this task's — `controller.rs` still holds the adaptation
> tier and its tests at this point, and removing them is fwloop.22's chartered work; this task
> only reshapes the type surface and updates call sites enough to compile.

`fwloop.22` is Task 12. Leave the adaptation tier alone. This task is **types only**.

This task is deliberately **dependency-free**: every new field is a plain type (`f64`, `u8`,
`String`, `Option<...>`, `&'static str`). Do **not** import `Budget`, `Curve`, `Arbiter` or any
other epic type into these definitions.

### The type surface this task owns

- `LoopMode { TempLoop, RpmLoop, Released }`
- `ControlStatus`: **drops** `trim_rpm`, `gain`; **adds** `mode`, `t_star_c: Option<f64>`,
  `ec_ma_c: Option<f64>`, `ec_argmax: Option<String>`, `duty_cmd: Option<u8>`,
  `snapped_rpm: f64`, `strategy: Option<String>`, `budget_w: f64`.
- `CalibProgressLite.phase` **stays a plain `String`**.
- `StatusFlag`: **drops** `ModelDistrust`; **adds** `FanctrlLost`, `EcMismatch`,
  `SteepCurve` (info), `CurveInvalid` (**warning** — a permanent loss of Mode A must not share
  the informational `SteepCurve` severity), `GpuHot`, `NvmeHot`, `ReadbackBlind`, each with a
  severity.
- `Effect::ModelSnapshot` **removed**.
- `Effect::AutoAllocated` **gains** `mode`, `error: f64`, `budget_w`,
  `freeze: Option<&'static str>`.

Existing code populates the new fields with defaults so the crate compiles.

### Acceptance criteria (verbatim from the bead)

> crate compiles and tests pass with the new type surface; `flag_severity` covers every new
> flag; the new `ControlStatus`/`Effect` definitions carry no `trim_rpm`/`gain`/`ModelSnapshot`
> field or variant.

### Implementation steps (TDD)

1. **Test first:** a test that constructs `ControlStatus` with the new field set. The old
   `trim_rpm`/`gain` call sites failing to compile is the signal that the fields are gone. Add
   `LoopMode` and the new fields.
2. **Test first:** `flag_severity` returns a severity for **every** `StatusFlag` variant — write
   it against a hand-maintained `const ALL: [StatusFlag; N]` and an exhaustive match, so adding
   a variant later fails to compile rather than silently defaulting. Assert specifically that
   `CurveInvalid` is **warning** and `SteepCurve` is **info**, and that the two differ. Then add
   the flags and their severities and delete `ModelDistrust`.
3. **Test first:** an `Effect::AutoAllocated` value carries `mode`, `error`, `budget_w` and
   `freeze`; `Effect` has no `ModelSnapshot` variant. Then change `Effect`.
4. Update `src/ui/view.rs` and `src/telemetry.rs` **minimally** — enough to compile. Where a
   removed field was rendered or serialised, drop that item; where a new field is needed for the
   match to be exhaustive, render/serialise it in the plainest possible way. Do not design the
   header segment here (Task 15).
5. Populate the new `ControlStatus` fields at existing controller call sites with defaults
   (`None` / `0.0` / `LoopMode::Released` as appropriate) so the build passes.
6. Run the test suite and the linter; both clean. Confirm the adaptation tier and its tests are
   **still present and still passing** — that is the evidence you stayed in scope.

### Deliverable

The crate compiles and the whole existing suite passes on the new type surface, with the
adaptation tier untouched.

---

## Task 5: Guards (dGPU, NVMe) + config keys

**Bead:** `fw-fanctrl-loop-mm2`

**filesTouched:** `src/control/guards.rs`, `src/config.rs`, `src/control/mod.rs`

`src/control/mod.rs` is a barrel: add exactly `pub mod guards;`.

### Global constraints

All of "Global Constraints" above applies. Normative: **§2.8**, plus §Facts's two measured
paragraphs (the card's 87 °C target specification; the NVMe airflow probe).

### The load-bearing negative result

**There is no `effective_target`.** The NVMe guard is **reporting-only** (§2.8, measured): the
airflow probe showed near-maximum airflow did not hold the drive while the SoC cooled, so
raising the fan target for a hot SSD buys nothing. Therefore **nothing modifies the user's RPM
target**, and `nvme_hot` only drives a flag. If you find yourself adding a function that returns
an adjusted target, stop — that is the deleted design.

`gpu_hot_c` defaults to **90** (exit 85), derived from the card's own 87 °C target
specification: any threshold below 87 fires during normal gaming.

### API this task owns

`Guards::step(gpu_temp_c: Option<f64>, nvme_temp_c: Option<f64>) -> GuardState { gpu_hot,
nvme_hot }` with enter thresholds, **exit = enter - 5**, and **`None` means that guard is
inactive (and it exits any hot state)**; `gpu_share_override(current_gpu_w, gpu_floor_w)`.
Config keys `gpu_hot_c` (90) and `nvme_hot_c` (80). The `online_rls` legacy note/test in
`config.rs` is generalised to "**unknown keys are ignored**".

### Acceptance criteria (verbatim from the bead)

> hysteresis enters at threshold, exits 5 below; a `None` input deactivates the guard and clears
> a hot state; the GPU share override is computed per spec §2.8; a hot NVMe sets `nvme_hot` and
> changes nothing else (no target, no budget); config round-trips with defaults; a config
> containing `online_rls`, a stale `nvme_boost_rpm` and an arbitrary unknown key still loads.

### Implementation steps (TDD)

1. Create `src/control/guards.rs`, add `pub mod guards;` to `src/control/mod.rs`.
2. **Test first:** `Guards::step(Some(90.0), None)` enters `gpu_hot`; it stays hot at 86.0 and
   clears at 85.0 (exit = enter - 5); symmetric table for `nvme_hot` at 80/75. Then implement
   the hysteresis.
3. **Test first:** a `None` GPU reading deactivates the guard **and clears an already-hot
   state**; same for NVMe. Then implement.
4. **Test first:** `gpu_share_override(current_gpu_w, gpu_floor_w)` returns the §2.8 value
   across the cases §2.8 enumerates. Then implement.
5. **Test first — the negative assertion:** a step whose only hot guard is `nvme_hot` returns a
   `GuardState` that carries **no** target or budget adjustment, and the module exposes no
   `effective_target` symbol. Assert `GuardState`'s field set explicitly.
6. **Test first:** `Config` round-trips with `gpu_hot_c` defaulting to 90 and `nvme_hot_c` to
   80. Then add the keys.
7. **Test first:** a config containing `online_rls`, a stale `nvme_boost_rpm` and an arbitrary
   unknown key loads successfully. Generalise the existing `online_rls` test and its comment to
   the "unknown keys are ignored" rule. Then implement (any `deny_unknown_fields` on this struct
   must be absent).
8. Run the test suite and the linter; both clean.

### Deliverable

`src/control/guards.rs` unit-tested standalone, plus the two config keys and the unknown-key
tolerance rule. No controller wiring.

---

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

## Task 7: fw-fanctrl socket client

**Bead:** `fw-fanctrl-loop-58u`

**filesTouched:** `src/fanctrl/client.rs`, `src/fanctrl/mod.rs`, `src/config.rs`,
`src/test_support/fakes.rs`

`src/fanctrl/mod.rs` — mod declaration only.

### Global constraints

All of "Global Constraints" above applies. Normative: **§2.1**, §3.4, §Facts (the two live
curves and `movingAverageInterval` 60).

**The read-only invariant is this task's to enforce structurally.** `PrintCommand { Speed, All }`
is the **only** command type the client can send. There is no escape hatch, no raw-string send,
no `set`/`use`/`pause` variant. A later task must not be able to write a fan speed through this
type even by mistake.

### What this task owns

`PrintCommand`, the `FanctrlSource` trait, `UnixFanctrlClient` (connect 1 s, read 3 s, raw CLI
string, read to EOF), `FakeFanctrl` in `src/test_support/fakes.rs` recording every command and
replaying scripted views/failures, `FanctrlView` (§2.1) with **two stamps** — `observed_at` (any
poll) and `all_observed_at` (last `print all`) — `curve` as the raw `Vec<(f64, u8)>` from
`resolve_curve(print_all_json, strategy)` (exact-name match in `strategies`),
`Freshness { Fresh, Stale, Absent }` with the **15 s (`print speed`) / 90 s (`print all`)**
rules, and the config key `fanctrl_socket` (default `/run/fw-fanctrl/.fw-fanctrl.commands.sock`).

### Acceptance criteria (verbatim from the bead)

> parses the fixtures (strategy, active, speed, temperature, ma interval); `resolve_curve` yields
> exactly the §Facts points for `quiet16` and `cool16`; ENOENT -> Absent; read timeout -> Stale;
> `print speed` failing for 15 s while `print all` is fresh -> Stale; a `print speed` refresh
> bumps `observed_at` but not `all_observed_at`; the fake's command log contains only
> `Speed`/`All`.

### Implementation steps (TDD)

1. Create `src/fanctrl/client.rs` and declare it in `src/fanctrl/mod.rs`.
2. **Test first:** parsing `tests/fixtures/fanctrl/print_all_quiet16.json` yields the strategy
   name, `active`, `speed`, `temperature` and `movingAverageInterval` 60. Then implement
   `FanctrlView` and the parser.
3. **Test first:** `resolve_curve(print_all_json, "quiet16")` yields exactly the §Facts quiet16
   points, and the same for `cool16`; an unknown strategy name yields no curve. Match the
   strategy name **exactly** — no fuzzy or case-insensitive matching. Then implement.
4. **Test first:** the two stamps — a `print speed` refresh bumps `observed_at` and leaves
   `all_observed_at` untouched; a `print all` bumps both. Then implement.
5. **Test first:** `Freshness` — `Fresh` inside both windows; `Stale` when the last `print all`
   is older than 90 s; `Stale` when `print speed` has been failing for 15 s **even while
   `print all` is fresh**; `Absent` on ENOENT. All timing is computed from the view's monotonic
   stamps. Then implement.
6. **Test first:** a read timeout maps to `Stale`, not `Absent`. Then implement the 1 s connect
   / 3 s read timeouts in `UnixFanctrlClient`.
7. **Test first:** `PrintCommand` has exactly two variants and `FanctrlSource` accepts nothing
   else; `FakeFanctrl`'s command log after a scripted session contains only `Speed`/`All`. Then
   implement `FakeFanctrl` in `src/test_support/fakes.rs` (replacing the empty stub from Task 2)
   with scripted views and scripted failures.
8. **Test first:** `Config` round-trips `fanctrl_socket` with the documented default. Then add
   the key.
9. Run the test suite and the linter; both clean.

### Deliverable

A read-only socket client plus a scriptable fake, both unit-tested on the Task 2 fixtures.

---

## Task 8: EC replica + NVMe + AC sensors

**Bead:** `fw-fanctrl-loop-52c`

**filesTouched:** `src/sensors/ec.rs`, `src/sensors/hwmon.rs`, `src/sensors/mod.rs`

### Global constraints

All of "Global Constraints" above applies. Normative: **§2.2**, §3.4, §Facts (the idle/load/
dGPU-on measurements).

### The measured fact this task must not contradict

With the dGPU powered at 18.9 W the cros_ec `gpu_amb`, `gpu_vr` and `gpu_vram` sensors still
read -150 and `gpu_temp@40` still returns ENODATA. They **never** report on this machine. The
replica's rule is nonetheless "every positive reading joins the max" — it matches fw-fanctrl's
own regex and costs nothing if a future firmware makes them live. Implement the general rule;
assert the measured fact.

### What this task owns

`EcReading` / `EcLabel` (controllable: `apu`, `cpu`, `gpu_*` if one ever reports; uncontrollable:
`ambient`, `charger`); every positive reading takes part in the max; drop readings at or below
zero, unreadable or unparsable `_input`; round to integer; max + argmax with ties broken by
**sysfs order**. `EcAverage`: boxcar of N non-zero samples **with the off-by-one**,
`set_interval(n)` capped at 100 and **retaining** existing samples, `reseed(value)` as the
**only** clearing operation, plus `is_seeded()` / `sample_count()` so the controller can refuse
to use an underfilled mean as a full one. `sensors/hwmon.rs` gains
`nvme_composite_c() -> Option<f64>` and `on_ac()`.

### Acceptance criteria (verbatim from the bead)

> on `cros_ec_idle` 47.85 -> 48, argmax `ambient`, the -150 and the input-less sensor dropped
> without invalidating the reading; on `cros_ec_load` 74.85 -> 75 with argmax `cpu@4c`, matching
> that fixture's paired socket `temperature` of 75.0; on `cros_ec_dgpu_on` the `gpu_*` sensors
> are still -150/absent and are dropped, so the max comes from `cpu`/`ambient` (measured: they
> never report even with the dGPU powered); a synthetic positive `gpu_*` reading would join the
> max and classify controllable; boxcar returns mean of n-N..n-1; `set_interval` grows/shrinks
> without clearing, `reseed` replaces the contents; `is_seeded` is false until seeded or N
> samples deep; nvme/ac readers on fixtures, nvme `None` when the chip is absent.

### Implementation steps (TDD)

1. Create `src/sensors/ec.rs` and declare it in `src/sensors/mod.rs`.
2. **Test first:** reading `hwmon/cros_ec_idle` yields max 48 (47.85 rounded) with argmax
   `ambient`; the three -150 sensors and the label-without-`_input` sensor are dropped and the
   reading stays valid. Then implement `EcReading` parsing.
3. **Test first:** reading `hwmon/cros_ec_load` yields 75 with argmax `cpu@4c`, and that equals
   the paired `print_all_load.json` `temperature` of 75.0. Then confirm the rounding rule.
4. **Test first:** reading `hwmon/cros_ec_dgpu_on` still drops every `gpu_*` sensor, so the max
   comes from `cpu`/`ambient`.
5. **Test first:** a **synthetic** positive `gpu_*` reading joins the max and classifies as
   **controllable**; `ambient` and `charger` classify uncontrollable. Then implement `EcLabel`
   and its controllability rule.
6. **Test first:** ties in the max are broken by sysfs order. Then implement.
7. **Test first:** `EcAverage` returns the mean of samples `n-N..n-1` — reproduce the off-by-one
   exactly, and write the test as a literal expected value so a later "fix" cannot silently
   change it. Then implement the boxcar over non-zero samples.
8. **Test first:** `set_interval` grows and shrinks **without clearing** retained samples and
   caps at 100; `reseed(v)` replaces the contents and is the only clearing operation. Then
   implement.
9. **Test first:** `is_seeded()` is false until `reseed` is called or N samples have arrived;
   `sample_count()` reports the retained count. Then implement.
10. **Test first:** `nvme_composite_c()` reads the `hwmon/nvme` fixture and returns `None` when
    the chip is absent; `on_ac()` reads `power_supply/ACAD/online`. Then implement in
    `src/sensors/hwmon.rs`.
11. Run the test suite and the linter; both clean.

### Deliverable

An EC replica whose max/argmax matches the socket on both captured fixtures, plus the boxcar
with its retain-on-resize contract, all unit-tested.

---

## Task 9: Actuator read-back

**Bead:** `fw-fanctrl-loop-jpg`

**filesTouched:** `src/actuators/cpu.rs`, `src/actuators/gpu.rs`, `src/control/controller.rs`

`src/control/controller.rs` — **the two actuator call sites only.** The RAPL stickiness watchdog
in `on_sample` is **untouched**.

### Global constraints

All of "Global Constraints" above applies. Normative: **§2.9**, §Facts (`ryzenadj --info` fails
while `ryzen_smu` is loaded; `nvidia-smi` reports `power.limit` N/A, so the GPU read-back is the
measured SM clock under load).

### What this task owns

`src/actuators/cpu.rs`: write `--slow-limit --stapm-limit --fast-limit`, then run
`ryzenadj --info` through the existing `Runner`, parse the `PPT LIMIT SLOW`, `PPT LIMIT FAST`
and `STAPM LIMIT` rows, and return
`WriteVerdict { Verified(w), Mismatch { field, commanded, read }, Unreadable, Unverifiable }`.
Slow and fast must agree within **0.5 W**; **STAPM is not required**; a failed `--info` yields
**`Unreadable`, never `Mismatch`** (that distinction is what stops a module-load precondition
from being read as a hardware fault).

`src/actuators/gpu.rs`: `verify_lock(gpu_util, gpu_sm_mhz) -> WriteVerdict` using the LUT-sweep
pin rule — util above 90 %, sm at most locked + 30, over 3 samples.

Existing controller callers compile by treating any non-`Verified` verdict as today's
success/failure until Task 19.

### Acceptance criteria (verbatim from the bead)

> parser on the fixture table; mismatch detected on a fake runner returning a stale table; a
> runner whose `--info` fails yields `Unreadable`; GPU verdict `Unverifiable` under 90 % util and
> `Mismatch` when the pinned clock exceeds lock + 30 for 3 samples; the existing RAPL watchdog
> tests still pass.

### Implementation steps (TDD)

1. **Test first:** the `ryzenadj --info` parser on `tests/fixtures/ryzenadj_info.txt` extracts
   the three named rows as watts. Then implement the parser.
2. **Test first:** commanding a value the fixture table agrees with (within 0.5 W on slow and
   fast) yields `Verified(w)`; STAPM disagreeing does **not** break verification. Then implement
   `set_sustained_mw`'s new return type.
3. **Test first:** a fake `Runner` returning a stale table yields
   `Mismatch { field, commanded, read }` naming the offending field. Then implement.
4. **Test first:** a fake `Runner` whose `--info` invocation fails yields **`Unreadable`** — and
   assert explicitly that it is not `Mismatch`. Then implement.
5. **Test first:** `verify_lock` returns `Unverifiable` when util is below 90 %; `Verified` when
   util is above 90 % and sm is at most lock + 30; `Mismatch` when the pinned clock exceeds
   lock + 30 across 3 samples (and not on 1 or 2). Then implement.
6. Update the two controller call sites to compile against `WriteVerdict`, mapping non-`Verified`
   to today's behaviour with a comment naming `fw-fanctrl-loop-j6s`. Do not touch the RAPL
   stickiness watchdog.
7. Run the test suite and the linter; both clean, **including the pre-existing RAPL watchdog
   tests**.

### Deliverable

Both actuators return a `WriteVerdict`, unit-tested on the fixture and on fake runners, with the
controller still behaving as before.

---

## Task 10: FOPDT fit + IMC gain derivation

**Bead:** `fw-fanctrl-loop-4aj`

**filesTouched:** `src/calib/fopdt.rs`, `src/calib/mod.rs`

### Global constraints

All of "Global Constraints" above applies. Normative: **§3.3** and **§2.4**.

### The theta trap — the single most important thing in this task

**Theta is taken as fitted.** The §3.3 fit runs on the **already-filtered** EC average, so its
`theta_hat` already contains the boxcar; adding `ma_interval/2` again would count the filter
twice and roughly **halve** `Kc`. The `+ ma_interval/2` substitution belongs **only** to the
raw-domain defaults of §2.4. If you write `theta_hat + ma_interval/2` anywhere in this file, the
task is wrong.

### What this task owns

`Fopdt { k, tau, theta }`, `fit_fopdt(&[(t, y)], step_w) -> Option<Fopdt>` by least squares on a
step response, and
`derive_gains(ec: &Fopdt, rpm: &Fopdt, defaults: &LoopGains) -> Option<LoopGains>` with
`lambda = max(90, 3*theta_hat)`, `Kc = tau/(K*(lambda + theta_hat))`, `Ti = tau` per signal, and
`fitted_at` left `None` for the caller to stamp.

**Rejection rules, all yielding `None`:** `K` at or below 0; `tau < 5 s`; a response magnitude
under 3 °C (EC) or 150 RPM (fan); a derived `Kc` outside `[0.25, 4]x` the corresponding default.
The magnitude and ratio bounds are what stop `Kc = tau/(K*(lambda+theta))` running away as `K`
approaches zero from above.

**Pure functions, no I/O.** No file reads, no clock reads.

### Acceptance criteria (verbatim from the bead)

> recovers K/tau/theta within 10 % on a synthetic noisy FOPDT step; derived gains match the IMC
> formulas **with the fitted `theta_hat` used as-is, no `+ ma_interval/2`** (the boxcar is
> already in `theta_hat`; adding it again halves `Kc`); each rejection rule fires on its own
> crafted input — `K <= 0`, `tau < 5 s`, a sub-threshold response, and a duty-pinned step whose
> tiny positive `K_rpm` would otherwise derive a `Kc` orders of magnitude above the default.

### Implementation steps (TDD)

1. Create `src/calib/fopdt.rs` and declare it in `src/calib/mod.rs`.
2. **Test first:** generate a synthetic FOPDT step (known K, tau, theta) with seeded noise;
   `fit_fopdt` recovers all three within 10 %. Then implement the least-squares fit.
3. **Test first — the theta assertion, written explicitly:** given a `Fopdt` with a known
   `theta_hat`, `derive_gains` produces `Kc = tau/(K*(max(90, 3*theta_hat) + theta_hat))`. Add a
   second assertion that the value is **not** what the `+ ma_interval/2` variant would give (it
   would be roughly half). Then implement `derive_gains`.
4. **Test first:** `Ti = tau` per signal, and `fitted_at` comes back `None`.
5. **Test first, one per rejection rule:** a non-positive `K`; `tau < 5 s`; an EC response under
   3 °C; a fan response under 150 RPM; and a **duty-pinned step whose tiny positive `K_rpm`**
   would otherwise derive a `Kc` orders of magnitude above the default, caught by the
   `[0.25, 4]x` band. Each returns `None`. Then implement each rule.
6. Run the test suite and the linter; both clean.

### Deliverable

Two pure functions with a complete rejection-rule test matrix. No calibration wiring.

---

## Task 11: Spike: settle the anti-windup rule

**Bead:** `fw-fanctrl-loop-9it`

**filesTouched:** `src/control/spike_antiwindup.rs`, `src/control/mod.rs`,
`docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md`

`src/control/mod.rs` — one `#[cfg(test)] pub mod spike_antiwindup;` line.
The design doc — **§2.4 only.**

### EPIC-SPECIFIC CONSTRAINT (settled outcome of three independent review rounds, not a suggestion)

> bead `fw-fanctrl-loop-9it` ("Spike: settle the anti-windup rule") is a SPIKE whose deliverable
> is a DECISION written into §2.4 of
> docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md, reached by measurement on its own
> throwaway harness. §2.4 deliberately fixes only the INVARIANTS — anti-windup is directional
> and may halt only the deepening direction, never pull u toward the measured draw; it judges
> per axis; any hold is visible — and deliberately leaves the predicate, the margins, the
> hysteresis and the GPU-HOT interaction UNSPECIFIED. Two earlier prose attempts (a
> back-calculation toward measured draw, which is a tracker rather than anti-windup; and a
> direction-blind freeze, which self-latches) were each independently confirmed Blocking.
> Therefore: (a) task `fw-fanctrl-loop-9it`'s plan section must state that the spike DECIDES
> those unspecified items by measurement and REWRITES §2.4 with the result, and must not present
> §2.4's current text as an implementable rule; (b) the plan section for `fw-fanctrl-loop-j6s`
> ("Controller loop integration"), which is blocked by 9it, must state explicitly that its
> implementer reads the REWRITTEN §2.4 from the integration branch and MUST NOT invent, infer,
> or reconstruct the anti-windup predicate, margins, hysteresis, or GPU-HOT interaction from
> prose — if §2.4 still reads as open when that task runs, that is a BLOCKED condition, not a
> licence to improvise.

**Read that again before starting.** §2.4's current text is **not an implementable rule** and
must not be treated as one. This task **decides** the predicate, the per-axis
`DEMAND_MARGIN_W`, the hysteresis/dwell/debounce, whether leaving calls `resync_error`, whether
the per-axis comparison uses the pre- or post-guard-override cap, and what a `GPU HOT` episode
does — **by measurement on the harness**, and then **rewrites §2.4** so the decided rule and its
constants are stated there as normative text. §2.4's "Open for the spike to decide and record"
list is **replaced** by the decision. The four fixed invariants stay; nothing this task decides
may violate them.

### Global constraints

All of "Global Constraints" above applies. Normative: **§2.4** (invariants), **§5** (the plant
constants), §Facts.

**This is a spike, not production code.** Its output is a decision plus a spec rewrite. The
harness is either deleted or kept as a `#[cfg(test)]` fixture — whichever the *result* argues
for; say which, and why, in your report.

### The harness

A throwaway rig around the **real** `Budget` (from Task 3): a first-order thermal plant
(tau 35, theta 20, K 0.8 per §5) **plus a demand model that decides how much of each commanded
cap is actually drawn**, so a cap can sit above the draw. Without that demand model the rig
cannot exercise the failure at all.

### The candidate rules to sweep

- no halt at all
- conditional integration halting **only the deepening direction**
- the same, with hysteresis / dwell
- per-axis versus combined

### The scenarios — every one the three roast rounds named

1. idle wind-up to the ceiling, then a load onset
2. a lull mid-session
3. a warm-start seeded from a heavier session, with a lighter load and the EC above T*
4. a mid-session target drop
5. a structurally undrawn axis (dGPU unpowered, CPU-only load)
6. a `GPU HOT` episode with the CPU at its own cap
7. oscillation around the margin when the draw sits near the cap

### Acceptance criteria (verbatim from the bead)

> every scenario above is run under every candidate and the results tabulated in the spec; the
> chosen rule holds the fan target in all of them with no self-latch and no cap-tracking;
> `DEMAND_MARGIN_W` is set per axis from the measured commanded-versus-drawn spread (RAPL
> against the slow-limit, NVML watts against the clock lock) rather than assumed, and that
> measurement is recorded in §Facts; §2.4's "open for the spike to decide" list is replaced by
> the decided rule.

### Implementation steps

1. Create `src/control/spike_antiwindup.rs` as a `#[cfg(test)]` module and declare it in
   `src/control/mod.rs`.
2. Build the rig: the real `Budget`, the first-order plant at tau 35 / theta 20 / K 0.8, and the
   demand model. Seeded RNG only — every run must be reproducible.
3. Encode the seven scenarios as data, and the four candidate rules as a small trait or enum, so
   the sweep is a cross product rather than seven hand-written cases per candidate.
4. **Measure `DEMAND_MARGIN_W` per axis on the real machine** — RAPL against the slow-limit for
   the CPU, NVML watts against the clock lock for the GPU. Record the measurement (numbers,
   method, date) in **§Facts**. Do **not** assume a value; if the machine measurement is
   genuinely unavailable, that is a BLOCKED condition to report, not a number to invent.
5. Run the full sweep. Tabulate candidate x scenario in the spec, with the criterion applied to
   each cell: does the fan target hold, is there a self-latch, is there cap-tracking?
6. Pick the winner. Verify it against the four fixed invariants explicitly — directional; never
   pulls `u` toward the draw; per-axis; the hold is visible.
7. **Rewrite §2.4**: replace the "Open for the spike to decide and record" paragraph with the
   decided rule stated normatively — the predicate, the per-axis `DEMAND_MARGIN_W`, the
   hysteresis/dwell/debounce, whether leaving calls `resync_error`, pre- or post-guard-override
   cap, and the `GPU HOT` interaction. Keep the four invariants and the "why prose failed"
   history; a reader arriving at §2.4 after this task must find an **implementable** rule, and
   must not be able to mistake the old open list for one.
8. Decide the harness's fate (delete, or keep as a `cfg(test)` fixture) and act on it.
9. Run the test suite and the linter; both clean.

### Deliverable

A rewritten §2.4 that states an implementable, measured anti-windup rule and its constants, plus
the sweep table that justifies it, plus the §Facts entry recording the `DEMAND_MARGIN_W`
measurement. Report the chosen rule and its constants **in your report text** as well, so the
coordinator can see the decision without opening the spec.

---

## Task 12: Remove the adaptation tier

**Bead:** `fw-fanctrl-loop-24s`

**filesTouched:** `src/control/controller.rs`

### Global constraints

All of "Global Constraints" above applies. Normative: **§3.2**, §4.

This task owns `src/control/controller.rs` **as a whole** for the duration — it is the file's
demolition pass, and Tasks 19 and 20 build on what it leaves. Every other controller-touching
task before it was restricted to named call sites precisely so this one can be a clean deletion.

### What to delete

The five-gate adaptation tier; the cooldown ring; the trust monitor; the `ModelSnapshot` period;
the degrade guard's `model.is_none()` check; the `persisted_bias` / `persisted_gain` plumbing;
and **every controller test that exercises them** — the KF, trust, cooldown and `fitted_model`
blocks.

**Do not delete the module files themselves** (`thermal_model.rs`, `kalman.rs`, `trust.rs`,
`cooldown.rs`, `trim.rs`) — Task 21 (`fw-fanctrl-loop-eyi`) owns that, and is blocked on this
task for it. Your job is to remove the last importer inside `controller.rs`.

After this task the auto path compiles with the budget stubbed at the floors and the existing
`AllocInput` call site (from Task 6).

### Acceptance criteria (verbatim from the bead)

> no import of `thermal_model`, `kalman`, `trust`, `cooldown` remains in the controller (source
> or tests); no `adapt_bias`/`adapt_gain`/`persisted_bias` symbol remains; `cargo test` green.

### Implementation steps

1. Inventory first: list every symbol and test block in `controller.rs` that belongs to the
   tier. Put the list in your report — it is what the reviewer checks the deletion against.
2. Delete the tier's production code, then its tests, in that order, so the compiler names any
   test you missed.
3. Remove the `model.is_none()` branch from the degrade guard and the `persisted_bias` /
   `persisted_gain` plumbing.
4. Confirm the surviving auto path still compiles against the floors-stubbed budget and the
   Task 6 `AllocInput` call site.
5. **Verify:** search `src/control/controller.rs` for `thermal_model`, `kalman`, `trust`,
   `cooldown`, `adapt_bias`, `adapt_gain`, `persisted_bias` — zero hits, in source and in tests.
   Record the search output in your report.
6. Run the test suite and the linter; both clean.

### Deliverable

A tier-free `controller.rs` with the full surviving suite green, and the five doomed modules
still on disk for Task 21.

---

## Task 13: Persisted state migration

**Bead:** `fw-fanctrl-loop-dsh`

**filesTouched:** `src/state.rs`, `src/control/controller.rs`

`src/control/controller.rs` — **the persist call sites only** (`save_persisted_state`,
`exit_auto_and_persist`, `apply_calib_effects`). Nothing else.

### Global constraints

All of "Global Constraints" above applies. Normative: §3.2, §4, §2.3 (the table's serde
default), §2.4 (`LoopGains`).

### The schema this task owns

    PersistedState {
        lut,
        calibrated_at,
        loop_gains: Option<LoopGains>,
        duty_rpm_table: DutyRpmTable,
        warm_start: BTreeMap<String, f64>,
    }

`model`, `adapt_bias` and `adapt_gain` are **removed**, along with the `state.rs` thermal-model
fit tests. **Old files must still load**: unknown keys ignored, missing new keys defaulted.
`DutyRpmTable`'s serde default (Task 1) is what makes a legacy file come back with the ten
seeded points rather than an empty table.

### Acceptance criteria (verbatim from the bead)

> the `state_v1.json` fixture loads with `lut` intact, `duty_rpm_table` equal to the ten seeded
> points, `warm_start` empty and `loop_gains` `None`; round-trip of the new schema; no
> `thermal_model` import remains in `state.rs`.

### Implementation steps (TDD)

1. **Test first:** loading `tests/fixtures/state_v1.json` (which contains `model` / `adapt_*`)
   succeeds, with `lut` intact, `duty_rpm_table` equal to the ten seeded points, `warm_start`
   empty and `loop_gains` `None`. Then reshape `PersistedState` and confirm the unknown-key
   tolerance.
2. **Test first:** a full round-trip of the new schema — write, read back, compare — including a
   populated `warm_start` and a `Some(LoopGains)`.
3. Delete `model`, `adapt_bias`, `adapt_gain` and the `state.rs` thermal-model fit tests.
4. Update the three persist call sites in `controller.rs` to the new shape. Nothing else in that
   file.
5. **Verify:** search `src/state.rs` for `thermal_model` — zero hits. Record it in your report.
6. Run the test suite and the linter; both clean.

### Deliverable

A migrated `state.json` schema that loads today's file without loss and round-trips the new
fields.

---

## Task 14: Sample plumbing + socket poller

**Bead:** `fw-fanctrl-loop-sov`

**filesTouched:** `src/types.rs`, `src/sensors/sampler.rs`, `src/sensors/mod.rs`, `src/main.rs`

`src/main.rs` — poller construction and shutdown join only.

### Global constraints

All of "Global Constraints" above applies. Normative: **§3.4**, §2.1, §2.2.

### The three-thread rule — the reason this task exists in this shape

- The **sampler** ticks at 1 Hz, reads EC and AC each tick, and merges the latest view.
- The **`FanctrlPoller`** runs on its own thread: `print speed` every 5 s, `print all` every
  30 s, **never faster**, sharing an `Arc<Mutex<...>>`.
- **The NVMe temperature is polled at 30 s on a thread of its own — neither the sampler tick nor
  the `FanctrlPoller`.** §3.4: a SMART admin read can block for the kernel's 60 s
  `admin_timeout`. On the sampler it would stall the control loop; on the poller it would fake a
  socket outage through the 15 s `print speed` staleness rule and drop the loop out of Mode A.
  It publishes a stamped last-good value; a missing or stale value surfaces as `None`.

**All freshness and cadence decisions are computed from the `Sample` / `FanctrlView` monotonic
timestamps.** The poller's own `Instant` drives only its cadence — never a freshness verdict.
No wall-clock anywhere in this path.

### What this task owns

`Sample` gains `ec: Option<EcReading>`, `ec_valid`, `nvme_temp_c: Option<f64>`,
`fanctrl: Option<FanctrlView>`, `fanctrl_freshness`, `fanctrl_view_changed: bool` (**true on the
first sample whose view `all_observed_at` differs from the previous sample's — a speed-only
refresh never sets it**), `on_ac`, and `resumed` (already produced by the existing resume
handler, now also consumed downstream). Plus the `FanctrlPoller` cadence, poller construction
from `Config::fanctrl_socket` with the real `UnixFanctrlClient`, and the shutdown join.

### Acceptance criteria (verbatim from the bead)

> with the fake source, over a 60 s scripted run the poller issues `Speed` at 5 s +/-1 tick and
> `All` exactly twice; `Stale` after 90 s of `All` failures and after 15 s of `Speed` failures;
> `fanctrl_view_changed` is true exactly once per new `All` view and never on a `Speed`-only
> refresh; **a scripted NVMe read that blocks for 60 s delays no `Sample`, leaves `Freshness`
> `Fresh` and the `print speed` cadence intact, and surfaces only as `nvme_temp_c: None`**; the
> real client is constructed from the configured path and both threads join on shutdown.

### Implementation steps (TDD)

1. **Test first:** a `Sample` carries every new field. Then extend `Sample` in `src/types.rs`.
2. **Test first:** over a 60 s scripted run against `FakeFanctrl`, the poller's command log shows
   `Speed` every 5 s (+/-1 tick) and `All` exactly twice. Then implement `FanctrlPoller` and its
   cadence.
3. **Test first:** `Freshness` becomes `Stale` after 90 s of `All` failures, and after 15 s of
   `Speed` failures. Then wire the freshness computation off the view's monotonic stamps.
4. **Test first:** `fanctrl_view_changed` is true on exactly the first sample after a new `All`
   view and **never** after a `Speed`-only refresh. Then implement.
5. **Test first — the NVMe isolation test, the one that justifies the third thread:** a scripted
   NVMe read that blocks for 60 s delays **no** `Sample`, leaves `Freshness` `Fresh`, leaves the
   `print speed` cadence intact, and surfaces only as `nvme_temp_c: None`. Then implement the
   30 s NVMe thread with its stamped last-good value.
6. **Test first:** the real `UnixFanctrlClient` is constructed from `Config::fanctrl_socket`, and
   both background threads join on shutdown. Then wire `src/main.rs`.
7. **Verify:** no wall-clock (`SystemTime::now`) appears in any freshness or cadence decision on
   this path. Record the search in your report.
8. Run the test suite and the linter; both clean.

### Deliverable

A 1 Hz `Sample` carrying the full new field set, fed by two independent background threads whose
blocking behaviour cannot reach the control loop.

---

## Task 15: TUI + telemetry surface

**Bead:** `fw-fanctrl-loop-mjv`

**filesTouched:** `src/ui/view.rs`, `src/telemetry.rs`, `src/model.rs`

### Global constraints

All of "Global Constraints" above applies. Normative: **§3.5**.

### What this task owns

`src/ui/view.rs`: the header segment `mode A|B|rel · T* · ma · duty -> rpm · budget`; rendering
and **ranking** of the new flags including `READBACK BLIND` (info); calibration progress renders
`CalibProgressLite.phase` as the plain string it already is (no enum, no mapping table).

`src/telemetry.rs`: sample fields `ec_max`, `ec_argmax`, `ec_ma`, `nvme_c`, `fanctrl_speed`,
`fanctrl_active`, `strategy`; decision fields `mode`, `t_star`, `budget_w`, `freeze`; **remove**
`trim_rpm`, `gain`, `model_*`.

### Acceptance criteria (verbatim from the bead)

> view snapshot tests for each mode and each new flag; a telemetry line serialises the new fields
> and omits the removed ones.

### Implementation steps (TDD)

1. **Test first:** a view snapshot for each of `TempLoop`, `RpmLoop` and `Released`, showing the
   header segment in the specified order. Then implement the header.
2. **Test first:** a view snapshot per new `StatusFlag` (`FanctrlLost`, `EcMismatch`,
   `SteepCurve`, `CurveInvalid`, `GpuHot`, `NvmeHot`, `ReadbackBlind`), and a ranking test that
   a warning-severity flag outranks an info one when both are present — specifically that
   `CurveInvalid` outranks `SteepCurve`. Then implement rendering and ranking.
3. **Test first:** calibration progress renders `phase` verbatim as the string it is, for both
   `"lut"` and `"step"`.
4. **Test first:** a serialised telemetry line contains every listed sample and decision field
   and contains **none** of `trim_rpm`, `gain`, `model_*`. Assert the absence explicitly, by
   substring, not only by struct shape. Then implement.
5. Update `src/model.rs` for whatever the view now needs.
6. Run the test suite and the linter; both clean.

### Deliverable

A header segment and flag surface with snapshot coverage for every mode and flag, and a
telemetry line matching §3.5 exactly.

---

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

## Task 17: fw-fanctrl emulator + chained plant

**Bead:** `fw-fanctrl-loop-51b`

**filesTouched:** `src/test_support/plant.rs`

This replaces the empty `cfg(test)` stub Task 2 created. It is one file, and it is the whole
task.

### Global constraints

All of "Global Constraints" above applies. Normative: **§2.5**, §5, and **§Facts** — especially
the EC-autofan staircase and its stated limitation.

**No new crate dependency.** The RNG is a hand-rolled seeded xorshift.

### The measured EC staircase, and the limit on what it may be used for

Measured medians: **4096 RPM at 61-62 °C, 4520 at 63 °C, 4658 at 64 °C, then flat at ~4748 RPM
across 67-73 °C.** The **67-73 plateau is well sampled (n = 102 at 71 °C) and load-bearing** for
the no-authority result. The **steep 61-64 segment is thin (n = 4 at 64 °C)** and its rising
branch and hysteresis width are **not** established (§Facts limitation). Model the plateau as
measured; treat the steep segment as **approximate**, and do not derive any claim about the EC
from it.

### The two upstream quirks that must be reproduced

These are what make the replica's average diverge from the socket's in ways the instantaneous
value hides — they are the whole reason `EC MISMATCH` exists:

1. **No history append while paused.**
2. **A hardcoded 50 °C injected on a scripted sensor-read failure.**

### The socket-death regime

The `Freshness` injection hook for socket death **also switches the plant into EC-autofan
mode** — a stopped fw-fanctrl leaves the fans on the EC curve via its unit's
`ExecStopPost --autofanctrl`, so `absent` and `active: false` are **one plant regime** (§2.5).

### What this task owns

`FanctrlEmulator` (1 s tick; boxcar N non-zero **with the off-by-one**; `eff = min(MA, cur)`;
`Curve`; `int()` truncation; switchable `active`; `edit_curve_in_place(points)` under the
unchanged strategy name; `view(now) -> FanctrlView` with curve, `ma_temperature`, `ma_interval`,
`active`, **both stamps**, plus the `Freshness` injection hook above).

`ThermalPlant` (watts -> controllable EC °C, tau 35, theta 20, K 0.8, plus separately labelled
`ambient` / `charger` channels and scriptable `gpu_*` channels emitted as an `EcReading`).

`FanPlant` (duty -> RPM via **its own table**, seeded from the same points with a **configurable
per-duty offset**; a one-sided momentum kick on positive slew; noise of +/-90 RPM from the seeded
xorshift; **plus an EC-autofan mode** used whenever the emulator is `active: false`, driving RPM
straight from EC temperature on the measured staircase).

Scriptable `gpu_temp_c` / `nvme_temp_c` (both `Option`), `cpu_util` / `gpu_util` / `gpu_sm_mhz`
tracks, **a demand model that decides how much of the commanded cap is actually drawn** (so
`cpu_pkg_w` / `gpu_w` on the emitted `Sample` are a **measured draw that can sit well below the
cap** — without this the plant cannot exercise the demand-starved wind-up at all), a scriptable
`resumed` edge, and `ChainedPlant` composing them into a full `Sample` per tick with
`fanctrl_view_changed` set **once per new `print all` view**.

### Acceptance criteria (verbatim from the bead)

> emulator reproduces the verified truncation case (T_eff 51.8 -> 21 on cool16) and the
> off-by-one; a scripted temperature drop drives `eff` from the `current` branch; a scripted
> socket death yields `Absent` **and puts the fan plant on the measured EC staircase**; an
> in-place curve edit yields new points under the same name in the emitted view;
> `fanctrl_view_changed` is set exactly once per new `print all` view; an open-loop step on the
> chained plant shows a 26-30 s watts->RPM lag; the plant table offset shifts steady RPM by the
> configured amount; scripted CPU-heavy vs GPU-heavy utilisation shifts the demand split; a
> low-demand script emits a measured draw well below the commanded cap; a paused emulator stops
> appending history, a scripted read failure injects 50 °C, and an `active: false` emulator
> drives RPM from the measured EC staircase instead of the commanded duty.

### Implementation steps (TDD)

1. **Test first:** the emulator reproduces `T_eff 51.8 -> 21` on cool16 (truncation) and the
   boxcar off-by-one, asserted as literal expected values. Then implement `FanctrlEmulator`'s
   tick, boxcar and `eff = min(MA, cur)`.
2. **Test first:** a scripted temperature **drop** drives `eff` from the `current` branch (not
   the MA branch). Then implement.
3. **Test first:** a paused emulator **stops appending history**; a scripted sensor-read failure
   injects **50 °C**. Then implement the two quirks.
4. **Test first:** `edit_curve_in_place(points)` yields the new points under the **same strategy
   name** in the emitted view, and bumps `all_observed_at`. Then implement.
5. **Test first:** `fanctrl_view_changed` is set exactly once per new `print all` view. Then
   implement the view emission.
6. **Test first:** an open-loop watts step on `ChainedPlant` shows a **26-30 s** watts-to-RPM
   lag. Then implement `ThermalPlant` (tau 35, theta 20, K 0.8) and the chaining.
7. **Test first:** the plant table's configurable per-duty offset shifts steady RPM by exactly
   that amount; the positive-slew momentum kick is one-sided; the noise is +/-90 RPM and
   reproducible from the seed. Then implement `FanPlant`.
8. **Test first:** a scripted socket death yields `Absent` **and** puts the fan plant on the
   measured EC staircase; an `active: false` emulator does the same. Assert the plateau values
   (4748 across 67-73 °C) exactly, and the steep segment only loosely. Then implement the
   EC-autofan mode.
9. **Test first:** scripted CPU-heavy versus GPU-heavy utilisation shifts the demand split; a
   low-demand script emits a **measured draw well below the commanded cap**. Then implement the
   demand model.
10. **Test first:** a scripted `resumed` edge appears on the emitted `Sample`.
11. Run the test suite and the linter; both clean.

### Deliverable

A deterministic, seeded `ChainedPlant` that emits a full `Sample` per tick and can reproduce
every regime Task 22's acceptance runs need — including the socket-dead / `active: false`
EC-autofan regime and demand starvation.

---

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

## Task 19: Controller loop integration

**Bead:** `fw-fanctrl-loop-j6s`

**filesTouched:** `src/control/controller.rs`

### EPIC-SPECIFIC CONSTRAINT — the anti-windup rule is NOT yours to invent

> bead `fw-fanctrl-loop-9it` ("Spike: settle the anti-windup rule") is a SPIKE whose deliverable
> is a DECISION written into §2.4 of
> docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md, reached by measurement on its own
> throwaway harness. §2.4 deliberately fixes only the INVARIANTS — anti-windup is directional
> and may halt only the deepening direction, never pull u toward the measured draw; it judges
> per axis; any hold is visible — and deliberately leaves the predicate, the margins, the
> hysteresis and the GPU-HOT interaction UNSPECIFIED. Two earlier prose attempts (a
> back-calculation toward measured draw, which is a tracker rather than anti-windup; and a
> direction-blind freeze, which self-latches) were each independently confirmed Blocking.
> Therefore: (a) task `fw-fanctrl-loop-9it`'s plan section must state that the spike DECIDES
> those unspecified items by measurement and REWRITES §2.4 with the result, and must not present
> §2.4's current text as an implementable rule; (b) the plan section for `fw-fanctrl-loop-j6s`
> ("Controller loop integration"), which is blocked by 9it, must state explicitly that its
> implementer reads the REWRITTEN §2.4 from the integration branch and MUST NOT invent, infer,
> or reconstruct the anti-windup predicate, margins, hysteresis, or GPU-HOT interaction from
> prose — if §2.4 still reads as open when that task runs, that is a BLOCKED condition, not a
> licence to improvise.

**Concretely, before you write the anti-windup wiring:**

1. Open `docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md` §2.4 **on the integration
   branch** (Task 11 rewrote it there).
2. Confirm it states, normatively: the predicate; `DEMAND_MARGIN_W` per axis; the
   hysteresis/dwell/debounce; whether leaving calls `resync_error`; whether the per-axis
   comparison uses the **pre-** or **post-**guard-override cap; and what a `GPU HOT` episode
   does.
3. If **any** of those still reads as open, undecided, or "for the spike to decide": **STOP and
   report BLOCKED.** Name which item is open. Do **not** invent, infer, or reconstruct it from
   the surrounding prose, from the invariants, or from the placeholder Task 3 left in
   `Budget::set_demand_state`. That placeholder is a seam, not a rule.

### Global constraints

All of "Global Constraints" above applies. Normative: **§2.4** (as rewritten), **§2.5**, **§2.6**,
**§2.9**, **§3.2**. This task builds on the tier-free `controller.rs` from Task 12.

### The data flow this task owns

`Budget::new(persisted loop_gains or default)` on auto entry, then `on_auto_sample`:

1. **Window pushes** — the existing fan window; `rpm_smoothed` = `FAN_SMOOTH_N` tail-mean;
   `fan_valid` from the sample.
2. **`EcAverage` push** — the controller owns the live instance: `set_interval` on view change,
   `reseed` on `Decision.reseed_ma`.
3. **Guards** (`Option` inputs) — `nvme_hot` raises its flag **only**.
4. **Every 5 s:**
   - budget bounds: `lo = cpu_floor_w + lut.watts_at(gpu_floor_mhz)`,
     `hi = cpu_max_w + gpu_max_w`, then `Budget::set_bounds`
   - `target_duty` from the user's target via `DutyRpmTable::duty_for_rpm` +
     `Curve::nearest_tread`
   - arbiter (`ArbiterInput` incl. `view_changed`, `at_lower_bound_for`, `at_upper_bound_for`,
     `error_sign`)
   - `resync_error` when `t_star_changed`
   - `LoopError` — Mode B's error is against `rpm_for_duty(target_duty)`, with the gain scaled by
     `scale_rpm_gain(slope)`
   - **the anti-windup halt exactly as the rewritten §2.4 states it** —
     `Budget::set_demand_state` with the per-axis smoothed draws, their caps and the error sign
   - `split_budget` (+ guard overrides + slew clamp)
   - CPU write + read-back verdict
   - GPU PI target at 1 Hz + `verify_lock` verdict

**The shared verdict rule for both actuators:** a candidate `Mismatch` is **re-read once** and
**suppressed for 3 ticks after an `on_ac` edge**; then `Mismatch` gives `LimitNotSticking` +
`Freeze::ActuatorMismatch` + an **immediate reassert on the same tick**; **three consecutive**
gives release to stock **with the flag held while the write + read-back keeps running every
reassert period, so a later `Verified` is producible** and clears the strikes.
`Unreadable` / `Unverifiable` are non-events; **six consecutive `Unreadable`** raises
`ReadbackBlind` until the next `Verified`.

`EcAverage` is seeded from `view.ma_temperature` on auto entry, on re-engagement, and at
calibration exit; it plus the fan and steady windows are **cleared on a `resumed` sample**.

A `Released` decision gives `release_to_stock` + `Freeze::Released` + flags; a later usable
decision re-engages — **seeding from the floors here**; the warm-start seed arrives in Task 20.

Auto entry requires `lut` **only**. Transitions emit `Noted { mode: ... }`. Status fields are
mirrored every tick. **The RAPL stickiness watchdog in `on_sample` is retained untouched.**

**Reconciliation scoring is at 1 Hz** — on the sample carrying the view, **not** on the 5 s
arbiter tick. The arbiter consumes the counters; this task owns when they are scored.

### Acceptance criteria (verbatim from the bead)

> controller unit tests on the fake seams cover: auto entry with LUT only; `Some(gains)` is
> loaded into `Budget`, `None` uses defaults; the integrator floor tracks a LUT change; an
> `NVME HOT` tick raises the flag and leaves `target_duty`, T* and the budget **unchanged**; a
> tick whose snapped duty the curve skips uses `nearest_tread`; a TempLoop tick computes
> `T* - MA` and moves the budget; an RpmLoop tick on socket Absent drives u from
> `rpm_for_duty(target_duty) - rpm_smoothed`; a same-name curve edit calls `resync_error` and
> produces no kick; a fan dropout clears `fan_valid` within one window; **the anti-windup
> scenarios `fwloop.24` decided, replayed at controller level against the rule it chose** — at
> minimum: an idle tick far below the cap neither winds to the ceiling nor decays toward the
> draw; a seeded-high budget with a lighter load still integrates **down**; a CPU-only tick with
> the dGPU unpowered keeps `u` off its lower bound; and a `GPU HOT` episode behaves as the spike
> specified; a scored view is evaluated on the 1 Hz sample carrying it, not on the 5 s arbiter
> tick; a rejected curve raises `CurveInvalid` and RpmLoop runs at the `0.25x` clamp; a CPU and a
> GPU `Mismatch` each freeze, flag and reassert on the same tick, and a mismatch within 3 ticks
> of an `on_ac` edge is suppressed; three consecutive mismatches release to stock with the flag
> held, the read-back keeps running, and a later `Verified` re-engages without a step; six
> `Unreadable` raise `ReadbackBlind` with no freeze; a `resumed` sample clears the boxcar and the
> steady window; a `Released` decision releases the caps, freezes, and a later usable decision
> re-engages without a step in u; a view change updates the boxcar interval without discontinuity
> in `ec_ma_c` and a `reseed_ma` decision re-seeds it; `Noted` transitions appear; the RAPL
> watchdog test still passes.

### Implementation steps (TDD)

1. **First, before any code:** perform the §2.4 check described in the EPIC-SPECIFIC CONSTRAINT
   above. Quote the decided rule and its constants into your report. If it is still open, stop
   and report BLOCKED.
2. **Test first:** auto entry with `lut` only succeeds; `Some(gains)` loads into `Budget` and
   `None` uses `LoopGains::default()`. Then implement entry.
3. **Test first:** the integrator floor tracks a LUT change (`lo = cpu_floor_w +
   lut.watts_at(gpu_floor_mhz)`). Then implement the bounds derivation.
4. **Test first:** `target_duty` snapping via `duty_for_rpm`, and a tick whose snapped duty the
   curve skips falls back to `nearest_tread`. Then implement.
5. **Test first:** a TempLoop tick computes `T* - MA` and moves the budget; an RpmLoop tick with
   the socket `Absent` drives `u` from `rpm_for_duty(target_duty) - rpm_smoothed`. Then implement
   the `LoopError` construction and the `scale_rpm_gain(slope)` application.
6. **Test first:** a same-name curve edit sets `t_star_changed`, calls `resync_error`, and
   produces no kick. Then wire it.
7. **Test first:** an `NVME HOT` tick raises the flag and leaves `target_duty`, T\* **and the
   budget unchanged**. Then wire the guards.
8. **Test first:** a fan dropout clears `fan_valid` within one window. Then wire the windows and
   `rpm_smoothed`.
9. **Test first — the anti-windup replay, against the decided rule only:** an idle tick far below
   the cap neither winds to the ceiling nor decays toward the draw; a seeded-high budget with a
   lighter load still integrates **down**; a CPU-only tick with the dGPU unpowered keeps `u` off
   its lower bound; a `GPU HOT` episode behaves **as §2.4 now specifies**. Then wire
   `set_demand_state` with the per-axis smoothed draws, their caps (pre- or post-guard-override
   **as §2.4 states**) and the error sign.
10. **Test first:** a scored view is evaluated on the 1 Hz sample carrying it, **not** on the 5 s
    arbiter tick. Then implement the §2.6 scoring cadence.
11. **Test first:** a rejected curve raises `CurveInvalid` and RpmLoop runs at the `0.25x` clamp.
12. **Test first, the verdict rule:** a CPU `Mismatch` and a GPU `Mismatch` each freeze, flag and
    reassert on the same tick; a mismatch within 3 ticks of an `on_ac` edge is suppressed; three
    consecutive release to stock **with the read-back still running**, and a later `Verified`
    re-engages **without a step**; six `Unreadable` raise `ReadbackBlind` **with no freeze**.
    Then implement the shared rule once, used by both actuators.
13. **Test first:** a `resumed` sample clears the boxcar and the steady window; a view change
    updates the boxcar interval **without a discontinuity in `ec_ma_c`**; a `reseed_ma` decision
    re-seeds it. Then wire the live `EcAverage`.
14. **Test first:** a `Released` decision releases the caps and freezes; a later usable decision
    re-engages **without a step in `u`** (seeded from the floors here). Then implement.
15. **Test first:** `Noted { mode }` transitions appear on each mode change; status fields are
    mirrored every tick.
16. Confirm the RAPL stickiness watchdog and its tests are untouched and still pass.
17. Run the test suite and the linter; both clean.

### Deliverable

A fully wired auto loop on the fake seams, whose anti-windup behaviour is exactly the measured
rule §2.4 states — quoted in your report — and never a reconstruction of it.

---

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

## Task 21: Deletion sweep

**Bead:** `fw-fanctrl-loop-eyi`

**filesTouched:** `src/control/thermal_model.rs`, `src/control/kalman.rs`,
`src/control/trust.rs`, `src/control/cooldown.rs`, `src/control/trim.rs`, `src/control/mod.rs`,
`src/control/gpu_pid.rs`, `TODO.md`

The five module files are **deleted**. `src/control/mod.rs` loses their `pub mod` lines.
`gpu_pid.rs` and `TODO.md` lose stray references.

### Global constraints

All of "Global Constraints" above applies. Normative: **§4** (deletions — "must sweep the whole
repo, source and non-source").

By the time this task runs, the importers are already gone: the controller's tier and tests with
Task 12, the `state.rs` fit tests with Task 13, the matrix code with Task 18, the telemetry
fields with Task 15, and `trim.rs`'s last user with Task 6. **This task removes whatever is
left** — stray imports, comments, docs and TODO mentions.

`README.md` is **not** yours — Task 23 (`fw-fanctrl-loop-7ij`) owns it.

### Acceptance criteria (verbatim from the bead)

> a repo-wide search for `thermal_model`, `kalman`, `trust::`, `cooldown`, `trim::`,
> `adapt_bias`, `ModelSnapshot`, `trim_rpm`, `contour`, `CONSERVATIVE_START`, `overshoot_settle`
> hits only `docs/research/`, `docs/plans/` history, this spec, and `README.md` (owned by
> fwloop.18); no `src/` hit remains; `cargo test` and `cargo clippy -D warnings` green.

### Implementation steps

1. **Search first, and record the full "before" output in your report.** Search the whole repo —
   source **and** non-source (docs, `TODO.md`, `README.md`, manifests) — for each of the eleven
   strings above **and** for each of the five module basenames.
2. Classify every hit: delete here, owned by Task 23 (`README.md`), or legitimately historical
   (`docs/research/`, `docs/plans/`, the spec itself).
3. Delete the five module files and their `pub mod` lines in `src/control/mod.rs`.
4. Remove the remaining references — `gpu_pid.rs` comments, stray imports, `TODO.md` mentions.
5. **Search again** and confirm the "after" output hits only `docs/research/`, `docs/plans/`,
   the spec, and `README.md`. **Zero `src/` hits.** Record the output in your report.
6. Run the test suite and the linter; both clean.

### Deliverable

A repo with no deleted-subsystem residue outside history docs and the README, with before/after
search output in the report.

---

## Task 22: Closed-loop acceptance + configuration smoke

**Bead:** `fw-fanctrl-loop-cm7`

**filesTouched:** `src/control/sim_tests.rs`, `src/control/mod.rs`

`src/control/mod.rs` — add exactly `#[cfg(test)] mod sim_tests;`.

### Global constraints

All of "Global Constraints" above applies. Normative: **§5** (the testing section, including the
period-agnostic relay rule). Every run is controller-level, on `ChainedPlant` (Task 17), through
the **real** `on_sample`, with Task 20's hooks active. The load step is a **utilisation + watts**
step, not a watts-only one. Seeded RNG throughout — every run must be deterministic.

### The two grading rules that are easy to get wrong

- **Relay detection is period-agnostic (§5):** no 3 or more consecutive sign-alternating band
  excursions **at any period**. Report the count and the dominant period for each run. Do not
  substitute a fixed-period oscillation check.
- **The calibration run's fit is graded against the boxcar-filtered plant it actually sees, not
  the raw tau 35 / theta 20 constants.** The criterion is that the derived `Kc` lands within
  25 % of the IMC value **for that filtered plant** and that the closed loop passes — **never**
  that tau/theta match the raw plant.

### The run list

**Baseline (4 runs):** `quiet16` and `cool16` x `TempLoop` and `RpmLoop` — load step then
30 min; at least 90 % of samples inside +/-150 RPM; no relay under the rule above.

**Robustness:** the same 4 runs with plant K/tau/theta perturbed +/-50 %.

**Refinement:** plant table biased -8 %; refinement brings RPM inside +/-150 within 20 min and
T\* follows the re-snapped duty.

**Demand-starved:** a long idle with the plant drawing far below the cap, then a load onset —
`u` never reaches the upper bound and the onset overshoot stays inside +/-150.

**Calibration:** a StepTest on the plant, then the `quiet16`/`TempLoop` acceptance repeated with
the fitted gains meeting the same bar, graded per the filtered-plant rule above.

**Transients:** a load release at t=1200 back inside +/-150 within 90 s; a dGPU-powered-and-hot
30 min run that stays in `TempLoop` with no `EC MISMATCH`; a dGPU-unpowered run (no `GPU HOT`,
floor honoured, `verify_lock` `Unverifiable`); a sub-floor target run raising
`TARGET UNREACHABLE (low)` and holding the floor without relay.

**Bumpless:** socket death at t=600 (A to B); `active: false` at t=700 with a fresh socket
(A to B); an in-place same-name curve edit at t=900. Each leaves |delta u| at most one increment
and the caps continuous.

**`active: false` authority run** (plant in EC-autofan mode, §Facts staircase): with a target
below the EC's flat band the loop parks `u` at the floor, raises `TARGET UNREACHABLE (low)`
naming the achievable RPM within 60 s, and the integrator does not wind; with the EC on its
steep segment below 64 °C the loop does not hunt. **The same run repeated with the socket
`Absent`** (the killed-daemon regime, §2.5) behaves identically and additionally raises
`FANCTRL LOST`.

**Demand-limited:** a duty-cycled load (5 min on, 2 min off, x3) must not let `u` decay toward
the lull draw and must return inside +/-150 RPM within 90 s of each onset; a CPU-only run with
the dGPU unpowered must leave `u` off its lower bound and `cpu_w` above `cpu_floor_w` after
10 min.

**Rejected curve:** a non-monotone curve keeps the loop in `RpmLoop` at the `0.25x` gain clamp
with `CURVE INVALID` raised, and **never** `SteepCurve`.

**`Released`:** socket absent **and** an invalid fan reading gives stock caps within one
hysteresis window, `FANCTRL LOST` + `SENSOR LOST` set; then sensor recovery re-engages `RpmLoop`
from the warm-start **without a cap step**.

**Faults:** feasibility (T\* below ambient + 5); the `high` unreachable case; the steep-curve
flag; a single read-back mismatch freeze; a mismatch suppressed across an `on_ac` edge; a
three-strike release followed by a `Verified` re-engagement; a 5 min `GPU HOT` episode at the
90 °C threshold with no post-episode overshoot above 150 RPM; an `NVME HOT` episode that raises
the flag **while the RPM trace is indistinguishable from the same run without it**;
reconciliation A to B to A with reseed; a scored view skipped because the replica was slewing;
and a `resumed` edge mid-run that clears the windows and writes **no** warm-start or refinement
across the gap.

**Global assertions over every run:** only `Speed`/`All` commands were ever recorded by the
fake; `ec_ma_c` tracks the emulator's `ma_temperature` within 1 °C in steady state; at least one
steady window is detected per converged run.

### Acceptance criteria (verbatim from the bead)

> all listed runs pass deterministically (seeded RNG); each spec-enumerated configuration (2
> strategies x 3 modes, dGPU on/off, default vs fitted gains) is exercised end to end (needs:
> fwloop.12, needs: fwloop.19).

### Implementation steps

1. Build the harness first: a run descriptor (strategy, mode, plant parameters, script,
   duration, seed) and a grader that returns the band-residency percentage, the relay count and
   the dominant period. Write the **grader's own unit tests** before any acceptance run — a
   broken relay detector silently passes everything.
2. Implement the **period-agnostic** relay rule and test it on synthetic traces: a clean
   converged trace (0 relays), a 3-alternation trace at one period, and a 3-alternation trace at
   a very different period (both must be caught).
3. Add the four baseline runs, then the +/-50 % perturbation set.
4. Add the refinement, demand-starved and calibration runs. For the calibration run, compute the
   IMC value **for the boxcar-filtered plant** and assert the derived `Kc` within 25 % of it —
   write that computation explicitly so nobody later "fixes" it to the raw constants.
5. Add the transient, bumpless, `active: false` authority (both socket-alive and `Absent`),
   demand-limited, rejected-curve and `Released` runs.
6. Add the fault matrix.
7. Add the three global assertions as a shared post-run check applied to **every** run.
8. Confirm the configuration coverage: 2 strategies x 3 modes, dGPU on/off, default vs fitted
   gains — write it as an explicit checklist test so a missing combination fails.
9. Run the test suite and the linter; both clean. Report each run's band residency, relay count
   and dominant period.

### Deliverable

A deterministic acceptance suite that grades the finished loop against §5, with the relay
detector itself unit-tested.

---

## Task 23: README + docs

**Bead:** `fw-fanctrl-loop-7ij`

**filesTouched:** `README.md`, `docs/research/03-control.md`,
`docs/superpowers/specs/INDEX.md`

### Global constraints

All of "Global Constraints" above applies. Normative: **§3.5**, §2.8, and the config keys as
they actually exist in `src/config.rs` at the time you run.

**Read the code, not this plan, for the names.** Config key names and defaults come from
`src/config.rs`; the flags table comes from the `StatusFlag` enum; the calibration flow
(durations, skip reasons) comes from `src/calib/step.rs`. If any of those disagrees with what
this section says, the code wins and you note it.

### What to write

- **Calibration walkthrough** — rewritten for the LUT sweep + step test, the burner, and
  `NeedsLoad`.
- **Auto mode** — the cascade, the modes, and a flags table.
- **Safety model** — fw-fanctrl owns the fans; the socket is read-only **by construction**;
  guards; read-back including `READBACK BLIND`; and a line saying **plainly that the NVMe
  reading is reported and not acted on**, with the measurement behind that (§2.8: near-maximum
  airflow did not hold the drive while the SoC cooled).
- **Configuration table** — drop `online_rls`; add `fanctrl_socket`, `gpu_hot_c`, `nvme_hot_c`.
- `docs/research/03-control.md` — a pointer note.
- `docs/superpowers/specs/INDEX.md` — status to implemented.

### Acceptance criteria (verbatim from the bead)

> every config key in `config.rs` appears in the README table and vice versa; every `StatusFlag`
> variant appears in the flags table; no README mention of matrix/model/trim/RLS remains.

### Implementation steps

1. Enumerate the actual `Config` keys from `src/config.rs` and the actual `StatusFlag` variants
   from `src/control/controller.rs`. Put both lists in your report.
2. Rewrite the Calibration walkthrough, Auto mode, Safety model and Configuration sections.
3. **Verify both directions of the config table:** every key in `config.rs` appears in the
   README, and every row in the README exists in `config.rs`. Do the same for the flags table.
   Record both checks in your report.
4. Search `README.md` for `matrix`, `model`, `trim`, `RLS` — zero mentions of the deleted
   subsystems remain (Task 21 deliberately left the README to you). Record the output.
5. Update `docs/research/03-control.md` and `docs/superpowers/specs/INDEX.md`.

### Deliverable

A README whose config and flags tables are provably in sync with the code, and no residue of the
deleted subsystems.

---

## Task 24: Integration sweep: fw-fanctrl closed loop

**Bead:** `fw-fanctrl-loop-nsc`

**filesTouched:** `src/integration_tests.rs`, `src/main.rs`,
`docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md`

`src/main.rs` — the `#[cfg(test)] mod integration_tests;` declaration, plus any small inline fix
the sweep turns up. The design doc — the **Post-Implementation Notes** section only.

This task runs last, on the merged tree, and is the epic's root integration sweep. Its
`filesTouched` deliberately understates the blast radius: **small inline fixes may land anywhere
under `src/`**. Keep each one small; anything larger becomes a filed bead, not a diff here.

### Global constraints

All of "Global Constraints" above applies. Normative: the whole design doc.

### The three jobs

**(1) Walk the goal's main flows end to end and implement what is missing.**

- Engage auto from a **fresh `state.json` with only a LUT**.
- Walk `TempLoop` -> socket death -> `RpmLoop` -> recovery -> `TempLoop`.
- Run a calibration.
- Restart the daemon and confirm the **warm-start, table and gains reload**.

**(2) Sweep for unwired config values, parameters and interfaces.** Every `Config` key is read
somewhere; every `StatusFlag` is raised somewhere **and** rendered; every telemetry field is
populated; every `Effect` variant is applied; every `CalibContext` field originates from live
data. This is the sweep that catches a key that was added and never consulted.

**(3) Add the integration tests no per-task test covers:** sampler -> controller -> telemetry
line with **real** types; config -> poller construction; a full `on_command` / `on_sample`
session on the fakes.

**Fix small gaps inline; file a blocker bead for large ones.** Do not absorb a large gap into
this task quietly.

### Acceptance criteria (verbatim from the bead)

> the three flows above pass on the fakes; the unwired-sweep checklist is recorded in the spec's
> Post-Implementation Notes with zero open items or a filed blocker per item; `cargo test` and
> `cargo clippy -D warnings` green.

### Implementation steps

1. Create `src/integration_tests.rs` (`cfg(test)`) and declare it in `src/main.rs`.
2. **Test first:** flow 1 — auto entry from a LUT-only `state.json`, the A -> B -> A walk, a
   calibration, then a restart that reloads warm-start, table and gains. Implement whatever is
   missing to make it pass.
3. **Test first:** flow 3's three integration tests — sampler to telemetry line with real types;
   config to poller construction; a full `on_command`/`on_sample` session on the fakes.
4. Build the **unwired-sweep checklist** as five explicit enumerations: every `Config` key,
   every `StatusFlag`, every telemetry field, every `Effect` variant, every `CalibContext`
   field. For each, show where it is read/raised/rendered/populated/applied. Prefer writing each
   enumeration as a **test** over writing it as prose, so it cannot rot.
5. Fix small gaps inline. For each large one, file a bead (`bd create`) and record its id.
6. Record the completed checklist in the spec's **Post-Implementation Notes**, with **zero open
   items** or a filed blocker id per item.
7. Run the test suite and the linter; both clean.

### Deliverable

A merged tree whose main flows are proven end to end, with a recorded sweep checklist that has
no open item without a bead id attached.

---

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

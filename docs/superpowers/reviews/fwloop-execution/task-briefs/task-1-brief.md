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


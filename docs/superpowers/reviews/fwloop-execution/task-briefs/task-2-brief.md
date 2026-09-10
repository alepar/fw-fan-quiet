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


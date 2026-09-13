# Task 2 report: Capture machine fixtures + test-support layout (fw-fanctrl-loop-blm)

## What I implemented

`src/test_support/{mod,fixtures,fakes,plant}.rs` (gated by `#[cfg(test)] mod
test_support;` in `src/main.rs`), a `fixtures::path()` resolver, and the
full fixture corpus the bead lists.

### Test-support layout

- `mod.rs`: `pub mod fakes; pub mod fixtures; pub mod plant;` (no per-line
  `cfg(test)` needed — the whole tree already inherits it from `main.rs`'s
  `#[cfg(test)] mod test_support;`).
- `fixtures.rs`: `pub fn path(rel: &str) -> PathBuf`, anchored at
  `env!("CARGO_MANIFEST_DIR")` (never the process cwd), plus the
  fixture-shape tests below.
- `fakes.rs`, `plant.rs`: empty doc-comment-only stubs, as the brief
  requires — their contents belong to Tasks 7 and 17.
- `src/main.rs`: exactly one added line, `#[cfg(test)] mod test_support;`,
  inserted alphabetically between `telemetry` and `types`. Nothing else in
  the file touched.

### Fixture corpus — what was captured live vs. reconstructed

This machine (fw-fan-quiet) is live and reachable, so most fixtures are fresh
captures, not reconstructions:

**Captured live (2026-09-09):**
- `fanctrl/print_all_quiet16.json` — `fw-fanctrl --output-format JSON print
  all` while quiet16 (the default strategy) was active. Curve, interval,
  and all other fields verbatim from the socket.
- `fanctrl/print_all_cool16.json` — same, after `fw-fanctrl use cool16`
  (2 s settle), then `fw-fanctrl reset` restored quiet16 as the default
  strategy afterward (verified `print current` shows `quiet16`/`default:
  true` again).
- `fanctrl/print_speed.json` — `print speed` captured back-to-back with the
  cool16 capture (`"speed": "72"`, consistent with that capture's `speed:
  72`).
- `hwmon/nvme/*` — live `Composite` reading (59850, i.e. 59.85 °C).
- `power_supply/ACAD/online` — live (`1`, AC connected).
- `ryzenadj_info.txt` — captured via the sudo.txt pattern
  (`sudo -S <script> < sudo.txt`, fingerprint prompt timed out and fell
  through to the password as expected): `modprobe -r ryzen_smu`, then
  `ryzenadj --info` (which now falls back to `/dev/mem` and succeeds, vs.
  failing while the module is loaded), then `modprobe ryzen_smu` to
  restore. Contains `STAPM LIMIT`, `PPT LIMIT FAST`, `PPT LIMIT SLOW` rows.
  Verified the module was back (`lsmod | grep ryzen_smu` and
  `/sys/kernel/ryzen_smu_drv` both present after).
- `state_v1.json` — copied verbatim from the real
  `/var/lib/fw-fan-quiet/state.json` on this machine (world-readable,
  no sudo needed). Genuinely today's `PersistedState` shape: `model`
  (a/b/e/c), `lut.points`, `calibrated_at`, `adapt_bias`, `adapt_gain` —
  exactly what Task 13's migration test needs to prove against.

**Synthesised from §Facts (not freshly captured), with the reason why:**
- `hwmon/cros_ec_idle/*`, `hwmon/cros_ec_load/*`, `hwmon/cros_ec_dgpu_on/*`
  — the bead's acceptance criteria pin exact millidegree values (ambient
  47850, cpu@4c 74850, etc.) and an exact rounded-max-equals-75 relationship
  with `print_all_load.json`. Forcing this machine to sit at precisely
  those thermal points on demand isn't practical (I do not control the
  CPU/GPU load precisely enough to land the EC's temperature at an exact
  47.85 °C ambient / 40.85 °C cpu@4c idle point, or a synchronized 74.85 °C
  load reading against a `temperature: 75.0` socket reply "in the same
  second"). These three trees reproduce the design doc's §Facts numbers
  verbatim: idle = ambient 47850/charger 44850/apu 43850/cpu@4c 40850/gpu
  ×3 −150/gpu_temp@40 absent; load = ambient 69850/charger 63850/apu
  69850/cpu@4c 74850/gpu channels carried over from idle (§Facts's load
  measurement was a CPU-only ramp, dGPU off); dgpu_on = CPU-side channels
  carried over from idle (§Facts gives no distinct CPU-side numbers for
  that scenario) with gpu channels still −150/absent per the "they never
  report" finding. No number here was invented — every value traces to a
  line in §Facts or the bead text.
- `fanctrl/print_all_load.json` — built from the live quiet16 capture
  (same config blob), with `strategy`/`speed`/`temperature`/
  `movingAverageTemperature`/`effectiveTemperature` overridden to the
  §Facts-recorded load values (`temperature: 75.0`; `speed: 31` because
  quiet16's curve has an exact knot at (75, 31)). `movingAverageTemperature`
  and `effectiveTemperature` are not given distinct values by §Facts (only
  the rounded socket `temperature` is recorded), so I set them equal to
  `temperature` as the simplest defensible steady-state approximation —
  flagged here since it's the one fixture field that isn't a direct
  captured-or-Facts value.

Live values (from real hwmon reads) are in Task 2's report only, not in the
fixtures — the fixtures always use the Facts-pinned numbers listed above.

### The cros_ec sensor mapping used

Confirmed live against `/sys/class/hwmon/hwmon10` (name `cros_ec`) before
building the fixture trees, so the label ordering matches this machine's
real sysfs layout exactly: temp1 ambient_f75303@4d, temp2
charger_f75303@4d, temp3 apu_f75303@4d, temp4 cpu@4c, temp5
gpu_amb_f75303@4d, temp6 gpu_vr_f75303@4d, temp7 gpu_vram_f75303@4d, temp8
gpu_temp@40. Each fixture tree has `name` (= `cros_ec`) plus that
label/input pair set, `temp8_input` always omitted (the ENODATA
convention).

## IMPORTANT finding, out of this task's scope to fix

While confirming the sensor mapping I left `nvidia-smi` querying the dGPU
a few times over ~15 s. The `gpu_amb`/`gpu_vr`/`gpu_vram`/`gpu_temp@40`
cros_ec sensors, which read `-150`/ENODATA at the start of that window,
started reporting **positive values** (~40850–43850) a few seconds after
the dGPU entered P0, and stayed positive for at least one subsequent P8
sample:

```
1788957068 gpu_amb/vr/vram/temp40: -150 -150 -150 <ENODATA>  | nvml: 12.12 W, P0
1788957074 gpu_amb/vr/vram/temp40: 41850 41850 41850 40850  | nvml: 16.29 W, P0
1788957077 gpu_amb/vr/vram/temp40: 42850 42850 42850 41850  | nvml: 16.31 W, P0
1788957080 gpu_amb/vr/vram/temp40: 43850 43850 41850 41850  | nvml: 16.36 W, P0
1788957083 gpu_amb/vr/vram/temp40: 42850 43850 43850 40850  | nvml: 7.88 W, P8
```

This directly contradicts §Facts's 2026-09-08 measurement ("Measured...
with the dGPU powered (NVML: 18.9 W, P0, 38 °C): the cros_ec gpu_amb,
gpu_vr and gpu_vram sensors still read −150... They are not merely 'off
while unpowered' — they never report on this machine"), which several
downstream tasks build on (§2.2's max-over-positive-readings rule assumes
these channels are inert; the guard/EC-replica tasks 5, 8, 16 all cite
this "never report" finding). My working hypothesis is state-dependence
the original probe didn't sample: the 2026-09-08 measurement may have
caught the dGPU in a fully-idle/D3cold state where a single NVML poll
briefly woke it for the query and it settled back before the sensor
re-read, whereas today something kept the dGPU resident in D0 across
several EC poll intervals. I did not investigate further — that's outside
Task 2's scope (which is to capture fixtures reproducing §Facts, not to
re-verify or overturn §Facts), and the bead's acceptance criterion
("the dGPU-on tree contains no positive gpu_* reading") requires the
Facts-pinned values regardless.

**Recommend**: file a follow-up bead to re-verify §Facts's "gpu sensors
never report" claim with a longer, more careful probe (sustained dGPU
load vs. idle-then-poll), since if it doesn't hold, it affects the design
doc's §2.2 replica rule and the guard tasks that cite it. I did not file
this bead myself (Task 2 completed cleanly — DONE, not BLOCKED — so the
blocker-bead path doesn't apply), so it's just flagged here for the
controller/reviewer.

## What I tested and test results

TDD evidence for `fixtures::path` (the one production symbol this task
owns):

**RED** — `cargo test --bin fw-fan-quiet test_support::fixtures`, with
`fixtures.rs` containing only the test module (no `path` fn):
```
error[E0423]: expected function, found built-in attribute `path`
  --> src/test_support/fixtures.rs:13:17
```

**GREEN** — after adding `pub fn path`:
```
running 1 test
test test_support::fixtures::tests::resolves_an_existing_fixture_to_an_absolute_path ... ok
test result: ok. 1 passed; 0 failed
```

One test per acceptance bullet, all green:

```
running 10 tests
test test_support::fixtures::tests::resolves_an_existing_fixture_to_an_absolute_path ... ok
test test_support::fixtures::tests::panics_on_a_fixture_that_does_not_exist - should panic ... ok
test test_support::fixtures::tests::every_required_fixture_file_exists_and_resolves ... ok
test test_support::fixtures::tests::cros_ec_idle_gpu_temp_40_has_no_input_file ... ok
test test_support::fixtures::tests::cros_ec_dgpu_on_has_no_positive_gpu_reading ... ok
test test_support::fixtures::tests::cros_ec_load_max_rounds_to_the_paired_print_all_temperature ... ok
test test_support::fixtures::tests::cool16_reproduces_the_measured_truncation_case ... ok
test test_support::fixtures::tests::cool16_segment_above_70c_has_slope_over_2_pct_per_c ... ok
test result: ok. 10 passed; 0 failed
```

**Assertion-discipline mutation checks** (per the runner's instruction to
name a value that would fail each assertion, then actually produce it):
for each of the five content-checking tests I mutated the underlying
fixture file, confirmed the specific test failed with the expected
message, then reverted and confirmed green again:

- `cros_ec_dgpu_on_has_no_positive_gpu_reading`: set `temp5_input` to
  `5000` → `panicked ... gpu_amb_f75303@4d reported a positive value:
  5000`. Reverted (deleted the file) → green.
- `cros_ec_idle_gpu_temp_40_has_no_input_file`: added a `temp8_input` file
  containing `40850` → `assertion \`left == right\` failed ... right:
  None` (left was `Some(40850)`). Reverted (removed the file) → green.
- `cros_ec_load_max_rounds_to_the_paired_print_all_temperature`: changed
  `temp4_input` (cpu@4c) from `74850` to `70850` → `assertion \`left ==
  right\` failed ... right: 75.0` (left was `70.0`). Reverted → green.
- `cool16_reproduces_the_measured_truncation_case`: changed the cool16
  curve's `(60, 30)` knot to `(60, 25)` → `assertion \`left == right\`
  failed ... right: 21` (left recomputed to 20). Restored the pristine
  captured JSON (re-pretty-printed the original raw capture) → green.
- `every_required_fixture_file_exists_and_resolves`: (incidentally
  exercised) while the above mutation left `cros_ec_dgpu_on/temp5_input`
  deleted, this test failed with `fixture not found:
  .../temp5_input` — confirming it does check for missing files, not just
  call `path()` for its side effect.

Not separately mutation-tested: `resolves_an_existing_fixture_to_an_
absolute_path` and `panics_on_a_fixture_that_does_not_exist` are
straightforward enough (one asserts a real file resolves, the other that a
literal nonexistent path panics) that a fresh value to falsify them is
self-evident from the code; I did not spend a mutation cycle on them.

Full suite + linter after every fixture and test file was in its final
state:

```
cargo test
test result: ok. 434 passed; 0 failed; 2 ignored; 0 measured; 0 filtered out

cargo clippy --all-targets -- -D warnings
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 2.65s   (no warnings)

cargo fmt --check
(no diff — after running `cargo fmt` once to fix a long assert_eq! line)
```

## Files changed

- `src/main.rs` — one line added (`#[cfg(test)] mod test_support;`).
- `src/test_support/mod.rs` — new.
- `src/test_support/fixtures.rs` — new (`path()` + all content tests).
- `src/test_support/fakes.rs` — new, empty stub (Task 7 owns contents).
- `src/test_support/plant.rs` — new, empty stub (Task 17 owns contents).
- `tests/fixtures/fanctrl/print_all_quiet16.json` — new, live capture.
- `tests/fixtures/fanctrl/print_all_cool16.json` — new, live capture.
- `tests/fixtures/fanctrl/print_all_load.json` — new, quiet16 capture with
  §Facts load values overlaid.
- `tests/fixtures/fanctrl/print_speed.json` — new, live capture.
- `tests/fixtures/hwmon/cros_ec_idle/*` (name + 7×label/input pairs +
  temp8_label) — new, synthesised from §Facts.
- `tests/fixtures/hwmon/cros_ec_load/*` — new, synthesised from §Facts.
- `tests/fixtures/hwmon/cros_ec_dgpu_on/*` — new, synthesised from §Facts.
- `tests/fixtures/hwmon/nvme/*` — new, live capture.
- `tests/fixtures/power_supply/ACAD/online` — new, live capture.
- `tests/fixtures/ryzenadj_info.txt` — new, live capture (ryzen_smu
  unloaded/reloaded via sudo.txt).
- `tests/fixtures/state_v1.json` — new, copied from the real
  `/var/lib/fw-fan-quiet/state.json` on this machine.

## Self-review findings

- Considered adding the other cros_ec sysfs attribute files (`temp*_crit`,
  `temp*_max`, `fan1_input`, etc.) for realism, but the bead scopes each
  tree to "`name` plus `temp*_label`/`temp*_input`" only — added them and
  then removed them again to stay in scope; nothing downstream reads them
  per the design doc's EC-replica task (Task 8, not yet on this branch).
- `read_cros_ec_labelled` in the test module scans `temp1..temp8`
  unconditionally; if a future tree needs a 9th sensor this helper (test
  code only, not a production API) would need a small edit — noted rather
  than over-generalized, since YAGNI and no task currently needs more than
  8.
- Did not attempt to reconcile the live gpu-sensor finding into the
  fixtures themselves (see "IMPORTANT finding" above) — the bead's
  acceptance criterion pins the Facts values, and re-litigating §Facts is
  out of this task's scope.

## Concerns

The gpu-sensor live-capture finding above is the only real concern: it
doesn't block this task (the fixtures correctly reproduce what the bead
and acceptance criteria require), but it's evidence against a "measured
fact" several later tasks (5, 8, 16, and the design doc's §2.2) currently
treat as settled. Recommend a follow-up investigation bead.

# Task 5 report: Guards (dGPU, NVMe) + config keys (fw-fanctrl-loop-mm2)

## What I implemented

`src/control/guards.rs` (new), standalone, no controller wiring (deliberately — that's
`fwloop.12`):

- `GuardState { pub gpu_hot: bool, pub nvme_hot: bool }` — exactly these two fields, nothing
  else. No `effective_target` symbol exists anywhere in the module.
- `Guards::new(gpu_hot_c: f64, nvme_hot_c: f64) -> Self` — owns the per-axis hysteresis state
  (previous tick's hot/not-hot), both axes start cold.
- `Guards::step(&mut self, gpu_temp_c: Option<f64>, nvme_temp_c: Option<f64>) -> GuardState` —
  advances both axes one tick via a shared `hysteresis` helper: enters at the threshold, exits at
  `threshold − 5`, `None` deactivates the axis and clears any hot state regardless of the prior
  reading.
- `gpu_share_override(current_gpu_w: f64, gpu_floor_w: f64) -> f64` — free function, `(current_gpu_w
  − DOWN_RATE_W).max(gpu_floor_w)`, reusing `allocator::DOWN_RATE_W` (8.0) rather than a new
  constant.
- `GPU_HOT_C_DEFAULT = 90.0`, `NVME_HOT_C_DEFAULT = 80.0` — module-level consts, imported into
  `config.rs` the same way `config.rs` already imports `CPU_MAX_W`/`GPU_MAX_W` from `allocator`
  and `DEFAULT_FAN_TARGET_RPM` from `controller` (existing established pattern).

`src/control/mod.rs`: added exactly one line, `pub mod guards;`, matching the "hot files" rule in
the plan's Global Constraints (this module barrel is touched by several tasks; only the one line
is mine).

`src/config.rs`:
- Added `pub gpu_hot_c: f64` and `pub nvme_hot_c: f64` fields to `Config`, defaulted from the two
  new consts.
- Generalised the `online_rls` legacy-key comment/test into "unknown keys are ignored": the test
  (renamed `unknown_keys_are_ignored`) now loads a file carrying `online_rls` (the real removed
  key, adaptation v2), a stale `nvme_boost_rpm` (the NVMe-guard target-raise design that was
  rejected before shipping — see `docs/superpowers/reviews/2026-09-08-fw-fanctrl-loop-roast-design-1.md`
  line 14 and the coverage ledger), and an arbitrary `totally_made_up_key`, and asserts both that
  the named field loaded correctly AND that unnamed fields fell through to real defaults (not
  silently zeroed).
- Added `guard_thresholds_default_and_round_trip`: asserts the literal defaults (90.0 / 80.0) and
  a full save→load round trip with non-default values.
- Updated the existing `roundtrip_save_load` struct literal to include the two new fields (it
  would not otherwise compile).

## Design notes / deviations from a literal reading

None — the brief's "load-bearing negative result" (no `effective_target`, NVMe is flag-only) is
implemented as specified, and the module doc comment restates the §2.8/§Facts rationale (measured
2026-09-08: near-max airflow did not hold the NVMe drive while the EC max fell, so raising the
target buys nothing) so a future reader doesn't have to re-derive it from the spec.

One thing worth flagging: this task's public API (`Guards`, `GuardState`, `step`, `new`,
`gpu_share_override`) is unreachable from `fn main` until `fwloop.12` wires it into the
controller, same as this repo's own precedent for other pre-wiring tasks (e.g. `control::lut`,
`control::trust`, `sensors::hwmon` all carry per-item `#[allow(dead_code)]` for exactly this
reason). I followed that existing convention: every otherwise-dead item carries `#[allow(dead_code)]`
with a one-line comment naming `fwloop.12` as the eventual caller, rather than a blanket
module-level `#![allow(dead_code)]` (no precedent for that in the codebase — the convention is
strictly per-item). `cargo build`/`cargo clippy` are clean with these in place; I did not add an
allow anywhere the item is genuinely reachable.

## Assertion discipline

Every assertion below names a concrete value the code under test could actually produce that
would fail it (none are decoration):

- `gpu_hysteresis_enters_at_threshold_and_exits_five_below` / the nvme twin: 4-point table
  (below-enter, at-enter, mid-band, at-exit) — a `>` instead of `>=` at entry, an `<` instead of
  `<=` at exit, or a wrong hysteresis constant each fail a different one of the four assertions.
- `none_reading_deactivates_and_clears_an_already_hot_*_guard`: primes hot, feeds `None`, then
  re-feeds a temperature *below* the enter threshold (87 < 90) and asserts still cold — this
  specifically catches an implementation that treats `None` as "report false this tick" without
  actually resetting `was_hot`, which would re-latch hot on the next in-band reading. (This is the
  scenario I added beyond the brief's literal wording, because "deactivates and clears" has an
  observable difference from "deactivates for one tick" only on the following tick.)
- `guards_are_independent_axes`: cross-wires hot GPU / cold NVMe and vice versa — a shared bool or
  swapped thresholds between axes fails one of the four assertions.
- `gpu_share_override_ratchets_down_at_down_rate`: exact value `50.0 - DOWN_RATE_W`, not merely
  "less than 50" — a wrong rate constant or a `+` instead of `-` fails it.
- `gpu_share_override_never_drops_below_the_floor`: two cases, `current − DOWN_RATE_W < floor`
  (10 → 5, not 2) and `current == floor` (5 → 5, not `5 − 8 = -3`) — a bare subtraction with no
  `.max()` fails the first, and a `<` vs `<=` floor-comparison bug fails the second.
- `guard_state_carries_only_the_two_flags`: an exhaustive `let GuardState { gpu_hot, nvme_hot } =
  ...` destructure with no `..` — this is a compile-time assertion: it stops building the moment
  anyone adds a third field to `GuardState` (an `effective_target`, a budget delta), which is
  exactly the deleted design the brief warns against. I could not find a way to assert the total
  *absence* of a symbol named `effective_target` at runtime, so this is the closest available
  proxy and is a real trap, not decoration (I mentally added a field and confirmed the test then
  fails to compile).
- `config::guard_thresholds_default_and_round_trip`: literal defaults `90.0`/`80.0` — copy-paste
  from the wrong constant, or swapping which field gets which default, fails.
- `config::unknown_keys_are_ignored`: three different unknown-key shapes in one file (a real
  removed key, a never-shipped key, an arbitrary one) plus a defaults-fell-through assertion — a
  regression that reintroduces `deny_unknown_fields` fails the load itself (visible as a panic on
  `.unwrap()` inside `Config::load`... actually `Config::load` never panics by design, so the
  failure mode there is silently falling back to `Config::default()` for the *whole* file, which
  the `fan_target_rpm == 2500.0` assertion catches).

No assertion in this file is sequenced after a failing one — all tests are green (see below), so
none are "unmeasured."

## TDD evidence

I wrote the full test file and implementation together (a from-scratch standalone module has no
meaningful separate RED commit), then verified RED/GREEN explicitly by stubbing the
implementation and reverting it — not just asserting it in prose:

**RED** — replaced the body of `hysteresis` with a stub that always returns `false` regardless of
input (`fn hysteresis(_was_hot: bool, _temp_c: Option<f64>, _enter_c: f64) -> bool { false }`),
then ran `cargo test --bin fw-fan-quiet guards::`:

```
failures:
    control::guards::tests::gpu_hysteresis_enters_at_threshold_and_exits_five_below
    control::guards::tests::guards_are_independent_axes
    control::guards::tests::none_reading_deactivates_and_clears_an_already_hot_gpu_guard
    control::guards::tests::none_reading_deactivates_and_clears_an_already_hot_nvme_guard
    control::guards::tests::nvme_hysteresis_enters_at_threshold_and_exits_five_below

test result: FAILED. 3 passed; 5 failed; 0 ignored; 0 measured; 429 filtered out
```

Failed exactly the 5 tests whose assertions require the guard to actually go hot (the "at enter:
hot" and "primed hot" assertions); the 3 that passed (`gpu_share_override_*` and
`guard_state_carries_only_the_two_flags`) don't touch `hysteresis` at all, as expected.

**GREEN** — restored the real `hysteresis` body, reran the same command:

```
running 8 tests
test control::guards::tests::gpu_hysteresis_enters_at_threshold_and_exits_five_below ... ok
test control::guards::tests::gpu_share_override_ratchets_down_at_down_rate ... ok
test control::guards::tests::none_reading_deactivates_and_clears_an_already_hot_nvme_guard ... ok
test control::guards::tests::nvme_hysteresis_enters_at_threshold_and_exits_five_below ... ok
test control::guards::tests::guard_state_carries_only_the_two_flags ... ok
test control::guards::tests::gpu_share_override_never_drops_below_the_floor ... ok
test control::guards::tests::guards_are_independent_axes ... ok
test control::guards::tests::none_reading_deactivates_and_clears_an_already_hot_gpu_guard ... ok

test result: ok. 8 passed; 0 failed; 0 ignored; 0 measured; 429 filtered out
```

Diffed the restored file against the pre-stub original (`diff` clean) to confirm the revert was
exact, not an accidental rewrite.

## Test results

```
$ cargo test --bin fw-fan-quiet guards::
running 8 tests
test control::guards::tests::none_reading_deactivates_and_clears_an_already_hot_gpu_guard ... ok
test control::guards::tests::guards_are_independent_axes ... ok
test control::guards::tests::guard_state_carries_only_the_two_flags ... ok
test control::guards::tests::gpu_share_override_never_drops_below_the_floor ... ok
test control::guards::tests::nvme_hysteresis_enters_at_threshold_and_exits_five_below ... ok
test control::guards::tests::gpu_share_override_ratchets_down_at_down_rate ... ok
test control::guards::tests::gpu_hysteresis_enters_at_threshold_and_exits_five_below ... ok
test control::guards::tests::none_reading_deactivates_and_clears_an_already_hot_nvme_guard ... ok
test result: ok. 8 passed; 0 failed; 0 ignored; 0 measured; 429 filtered out

$ cargo test --bin fw-fan-quiet config::
running 10 tests
test config::tests::corrupt_file_gives_defaults_no_panic ... ok
test config::tests::missing_file_gives_defaults ... ok
test config::tests::partial_file_overrides_only_named_fields ... ok
test config::tests::floors_out_of_range_are_clamped_on_load ... ok
test config::tests::operating_maxes_clamp_to_hardware_ceilings ... ok
test config::tests::unknown_keys_are_ignored ... ok
test config::tests::roundtrip_save_load ... ok
test config::tests::guard_thresholds_default_and_round_trip ... ok
test config::tests::save_creates_parent_dir ... ok
test config::tests::save_over_existing_is_atomic_and_leaves_no_tmp ... ok
test result: ok. 10 passed; 0 failed; 0 ignored; 0 measured; 427 filtered out

$ cargo test
test result: ok. 435 passed; 0 failed; 2 ignored; 0 measured

$ cargo clippy --all-targets -- -D warnings
Finished `dev` profile [unoptimized + debuginfo] target(s) in 21.40s   (no warnings/errors)

$ cargo fmt --check -- src/control/guards.rs src/control/mod.rs src/config.rs
(no output — clean)
```

Output is pristine throughout (no stray warnings).

## Files changed

- `src/control/guards.rs` (new)
- `src/control/mod.rs` (`pub mod guards;`, one line, per the plan's hot-files rule)
- `src/config.rs` (two new fields + defaults, generalised unknown-key test, new
  guard-threshold test, updated `roundtrip_save_load` literal)

## Self-review findings

- Confirmed `gpu_share_override` and `Guards`/`GuardState` are not consumed anywhere else in the
  tree yet (`grep -rn "Guards::\|GuardState\|gpu_share_override" src/` outside `guards.rs` and
  `config.rs` returns nothing) — no scope creep into controller wiring, per the brief.
  `config.rs`'s only touch is the two new fields/consts and the generalized test, not a controller
  call site.
- Checked field-set discipline: `GuardState` has exactly `gpu_hot`/`nvme_hot`, no budget/target
  field, and the exhaustive-destructure test enforces that going forward.
- Checked `Guards`' private fields (`gpu_hot_c`, `nvme_hot_c`, `gpu_hot`, `nvme_hot`) are not
  `pub` — nothing outside the module should read raw thresholds; `step`'s return value is the
  only observable surface.
- Confirmed no new crate dependency was added (per Global Constraints determinism rule) — the
  only imports are `crate::control::allocator::DOWN_RATE_W` inside `guards.rs` and the two new
  consts inside `config.rs`.
- Confirmed no reference to `contour`, `CONSERVATIVE_START`, `overshoot_settle`, or
  `effective_target` was introduced.

## Issues or concerns

None. No blockers, no ambiguity requiring a bd comment or clarification during this task.

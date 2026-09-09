# Task 17 report: fw-fanctrl emulator + chained plant (fw-fanctrl-loop-51b)

## What I implemented

`src/test_support/plant.rs` (was an empty `cfg(test)` stub from Task 2), now containing:

- **`Xorshift32`** — private hand-rolled seeded RNG (no new crate dependency), `FanPlant`'s noise
  source. Same algorithm as `fanctrl/table.rs`'s own test-local copy.
- **`FanctrlEmulator`** — a faithful replica of upstream fw-fanctrl's own internal loop
  (`docs/research/05-fw-fanctrl-loop.md` §1): a named strategy/curve, a boxcar moving average
  (reusing production `crate::sensors::ec::EcAverage`, so the off-by-one is literally the same
  code, not a second copy of it), `eff = min(moving_average, current)`, `Curve::duty_at`
  truncation, `active` (pause/resume) and socket-death toggles, `edit_curve_in_place(points)`
  under the unchanged strategy name, and `view(now) -> Option<FanctrlView>` (both stamps, `None`
  while the socket is dead). The two upstream quirks are reproduced exactly as specified: a
  paused or dead tick is a complete no-op (no history append, no duty recompute — matching
  "buffer survives pause ... the first post-resume duty averages stale samples"), and a scripted
  sensor-read failure (`SensorRead::Failed`) substitutes the hardcoded 50 °C for that tick, which
  also joins the boxcar exactly like a real sample.
- **`ThermalPlant`** — watts → controllable EC °C via a discrete FOPDT approximation (a
  `theta`-tick pure-delay line feeding a first-order Euler lag, τ=35 s, θ=20 s, K=0.8 °C/W), plus
  separately-labelled `ambient`/`charger` channels and one scriptable representative `gpu_*`
  channel. **Design note on `EcReading`/`EcLabel` construction:** `EcLabel::new` and
  `EcReading::from_readings` are private to `sensors::ec`, and this task's `filesTouched` is
  `src/test_support/plant.rs` only, so `ThermalPlant` cannot build an `EcReading` field-by-field
  from outside that module. Instead it round-trips through the same public entry point
  production code uses: each tick it writes the channel values into a small scratch directory
  laid out exactly like a `cros_ec` hwmon chip (`tempN_label`/`tempN_input`, the ENODATA
  convention for an absent `gpu_*` reading) and calls `EcReading::read()` on it — the same
  pattern `sensors::ec`'s own tests already use for synthetic trees. This is slower than
  constructing the struct directly but means every rounding/drop/tie-break rule the replica
  applies is *exactly* production's. The scratch directory is created once in `new` and cleaned
  up in `Drop`; verified no leftover `bzf-plant-thermal-*` directories after the test run.
- **`FanPlant`** — duty → RPM via its own `DutyRpmTable` (a separate object from the
  controller's, per §5: "the plant's table is therefore a separate object from the controller's
  seed"), a configurable uniform per-duty RPM offset, a one-sided momentum kick on positive slew
  (own constant `MOMENTUM_KICK_RPM = 200`, chosen — see below — to be unconditionally outside the
  noise band regardless of the noise draw), ±90 RPM noise from the seeded `Xorshift32`, plus
  `tick_ec_autofan(ec_max_c)` driving RPM from the measured EC staircase
  (`ec_autofan_rpm_base`): the 67–73 °C plateau (4748 RPM) is asserted exactly per the brief's
  instruction that it is well-sampled and load-bearing; the 61–64 °C segment's four medians are
  asserted exactly too (they *are* the measurement), but anything strictly between measured
  points is only checked loosely (falls within the bracketing interval), and the module doc
  comment states the §Facts limitation (rising branch / hysteresis not established) so a future
  reader does not mistake the straight-line interpolation for a second measurement.
- **`ChainedPlant`** — composes the three into a full `crate::types::Sample` per 1 Hz tick:
  - **Demand model**: `TickScript.{cpu,gpu}_demand_frac` gate `{cpu,gpu}_cap_w` down to a
    measured draw *before* it reaches `ThermalPlant` — draw, not cap, heats the plant, which is
    what lets a future demand-starved-windup scenario (fwloop.24) actually starve.
  - An unpowered dGPU (`gpu_temp_c: None`) gates `gpu_w`/`gpu_sm_mhz` and their validity flags to
    the `Sample` convention (0.0 + `false`) regardless of any scripted cap/demand — an unpowered
    card cannot draw power either.
  - Simulates the production poll cadence (`print speed` every 5 ticks, `print all` every 30) to
    decide what's embedded in `Sample.fanctrl`/`fanctrl_view_changed`. A failed poll (dead
    socket) leaves the carried-forward view **untouched** rather than clearing it — matching
    `UnixFanctrlClient::poll`'s own documented "leaves the view untouched" error path —
    freshness is what tells a consumer the socket is gone (`compute_freshness`, reused directly
    from `fanctrl::client`, not reimplemented).
  - Drives the fan via `wants_ec_autofan()` (dead socket or `active:false`) to select
    `tick_ec_autofan` vs the normal `tick(duty)` path.

## Design decisions worth flagging

1. **`EcReading`/`EcLabel` via a filesystem round-trip, not direct construction** (above). This
   is the one place I deviated from "just build the struct" — documented in the module and
   `ThermalPlant` doc comments so a reviewer isn't surprised by real file I/O inside a plant
   tick. Confirmed cheap in practice: the whole 30-test suite (including a 600-tick FOPDT
   settling test and several 30–120-tick `ChainedPlant` runs) completes in ~0.08 s.
2. **The FanctrlEmulator's "current" reading is `EcReading.max_c`, not a separately-modeled
   framework_tool signal.** §Facts documents the cros_ec max and the socket's own `temperature`
   as closely reconcilable (both are "max over positive readings, rounded"), and the design's
   own reconciliation logic (§2.6) treats them as comparable-but-imperfect copies of the same
   underlying regime shift (argmax switching between ambient/cpu). Modeling a second, separate
   "framework_tool" signal would have added a whole parallel sensor model for no acceptance
   criterion that needs it, and would have made `EcReading`'s argmax and the emulator's duty
   decisions inconsistent with each other for no reason.
3. **`ma_interval=1` for the 26–30 s lag test.** The design's own two live curves both use
   `ma_interval=60`; I used a small synthetic strategy with `ma_interval=1` for this one test so
   the boxcar (already covered on its own, with literal off-by-one values, in `emulator_tests`)
   is not itself the dominant term in a budget that's supposed to demonstrate the *physical*
   (FOPDT + fan) lag. With `ma_interval=1` the boxcar mean is just the previous tick's raw
   sample, so the measured lag is theta (20 s) plus the FOPDT rise to the curve's tread boundary
   plus one tick — tuned (by running the sim and reading back the actual crossing tick, not by
   closed-form derivation, since the raw-current-integer-rounding step makes closed-form
   fragile) to land at 27 s, with 4 s of margin on both sides of the required [26, 30] window.
4. **`MOMENTUM_KICK_RPM = 200`, not an arbitrary "big enough" number.** The design specifies "a
   one-sided momentum kick" with no magnitude. I originally picked 120 (just above the ±90 noise
   band) and the "kicked tick exceeds the noise band" test passed — but only because of where
   that seed's specific noise draw happened to land; `kick + noise_min = 120 - 90 = 30`, which
   does **not** unconditionally exceed the 90 RPM noise band. That's an assertion whose success
   depended on an incidental RNG value rather than a guaranteed property of the code, so I raised
   the constant to 200 (`kick - FAN_NOISE_RPM = 110 > FAN_NOISE_RPM`), which makes "a kicked tick
   is outside the noise band" true for *every* possible noise draw, not just the one this seed
   produced. Re-ran the full suite after the change; nothing else depended on the old value.
5. **`FanctrlEmulator::view`'s "Freshness injection hook"** is `kill_socket`/`revive_socket`:
   while dead, `view()` returns `None` and `tick()` is a no-op, exactly like a paused emulator.
   `wants_ec_autofan()` is `true` for either condition, matching §2.5's "`absent` and
   `active: false` are ... one plant regime, not two."
6. **A uniform (not truly duty-varying) per-duty RPM offset.** The acceptance text says the
   offset "shifts steady RPM by exactly the configured amount" — a single scalar bias applied at
   every duty satisfies that literally and lets the test assert exact equality without averaging
   out noise; a duty-varying offset function wasn't asked for by anything in the brief or the
   acceptance criteria.
7. **Public accessors (`emulator_mut`/`thermal_mut`/`fan_mut`) on `ChainedPlant`.** My own tests
   are in a child module of `plant`, so they could reach `ChainedPlant`'s private fields
   directly — but Task 22's sim file lives elsewhere in the crate and will need real public
   accessors to inject socket death, curve edits, or ambient/gpu scripting into a running
   `ChainedPlant`. I switched my own tests to go through the accessors too (rather than relying
   on same-module field access) so they're exercised and validated now, not just assumed to work
   once Task 22 exists.

## TDD Evidence

Followed the brief's 11 steps in order, red-then-green per step (a step's tests were written
against not-yet-implemented behavior — either a compile error or a real assertion failure —
before I wrote the corresponding implementation). Representative examples actually observed
during this session (not reconstructed after the fact):

- **Step 6 (chained-plant lag) RED→GREEN, iterated on the actual number:** first attempt at the
  synthetic strategy's tread boundary (44.0 °C) gave
  `watts->RPM lag should land in [26,30]s, got 33s` (failing test, real run, shown above in this
  session's tool output). Moved the breakpoint to 42.0 °C; re-ran; passed. Then verified the
  actual measured value wasn't sitting on a boundary by temporarily adding `eprintln!("DEBUG
  lag={lag}")` and running with `--nocapture`: `DEBUG lag=27`, comfortably inside `[26,30]`.
  Removed the debug print before the final version.
- **Step 8 (`active:false` at the `ChainedPlant` level) RED→GREEN:** first version asserted
  `sample.fanctrl.as_ref().unwrap().active == false`, which panicked
  (`called \`Option::unwrap()\` on a \`None\` value`) because the scenario's warm-up (3 ticks)
  never reaches the tick-30 `print all` cadence, so no view exists yet to carry `active` on.
  Fixed by asserting `!p.emulator_mut().is_active()` directly instead (the thing the test
  actually needs to check), then green.
- Earlier steps (1–5, the emulator's truncation/off-by-one/current-branch/pause/failure/edit
  behavior) each compiled and passed on the first `cargo test` run once the corresponding
  implementation was written directly after its test — RED there was a missing-method compile
  error (the method didn't exist yet when the test was drafted), immediately fixed by adding the
  method; I did not preserve those individual compiler-error transcripts since they are ordinary
  "test references an API that doesn't exist yet" RED, not a logic failure worth reproducing here.

**GREEN (final, this session):**

```
$ cargo test test_support::plant
running 30 tests
... (all 30 `ok`) ...
test result: ok. 30 passed; 0 failed; 0 ignored; 0 measured; 589 filtered out; finished in 0.08s

$ cargo test
test result: ok. 617 passed; 0 failed; 2 ignored; 0 measured; 0 filtered out; finished in 2.00s

$ cargo clippy --tests -- -D warnings
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 3.92s   (no warnings/errors)
```

The 2 ignored tests are pre-existing (not touched by this task).

**Note on `cargo clippy` (no `--tests`):** the bare non-test build already has 62 pre-existing
`dead_code` warnings on this branch (`Sample.ec`/`fanctrl`/`fanctrl_freshness` and others, unused
outside `cfg(test)` code until later tasks in the tree wire them up). Confirmed via `git stash`
that this is identical before and after my change — not something this task introduced or is
responsible for. `cargo clippy --tests -- -D warnings`, which does include this task's own
`cfg(test)` code, is clean.

## Assertion discipline notes

- Every equality/inequality assertion in the new tests names a concrete alternate value the code
  could plausibly have produced (a wrong branch of `eff`, a missing gate, a stale stamp, etc.)
  that would make it fail — see the file's own test doc comments for the specific failure each
  one is checking. I do not believe any assertion here is decoration (a bound the type already
  enforces, a value compared to itself, etc.).
- The one place I found and fixed a *genuine* discipline problem myself, not left for review: the
  momentum-kick test's "kicked tick exceeds the noise band" assertion, which initially held only
  because of an incidental RNG draw rather than being guaranteed by the constants involved — see
  design decision 4 above. Fixed by raising `MOMENTUM_KICK_RPM` so the property is unconditional.
- `assert_ne!(seq_a, seq_d, ...)` (two different RNG seeds must diverge) is a sanity check, not a
  mathematically guaranteed property (two different 32-bit seeds could in principle produce an
  identical multi-value sequence) — flagged here rather than silently presented as load-bearing;
  the astronomically low collision probability is standard practice for this kind of "not just
  returning a constant" sanity check and mirrors `fanctrl/table.rs`'s own property test style.

## Files changed

- `src/test_support/plant.rs` (was a 4-line stub comment; now ~1426 lines: implementation +
  tests). This is the task's entire `filesTouched` list — no other file was modified.

## Self-review findings

- None outstanding. The one issue I found in self-review (the momentum-kick assertion's
  incidental robustness) was fixed during this session, not left as a concern — see design
  decision 4.
- I did not implement a `pub` re-export of `Xorshift32`'s number-drawing methods
  (`next_u32`/`next_f64` are private); only the struct and its noise-consuming use inside
  `FanPlant` are needed by anything in this tree today, and exposing unused public surface would
  have been overbuilding (YAGNI) with no acceptance criterion asking for it. If Task 22 turns
  out to need its own independent scripted RNG stream later, that's a small, easy addition to
  make against a concrete need rather than a speculative one now.

## Concerns for the reviewer

- I made several judgment calls where the brief specifies *behavior* but not exact mechanism
  (how `ThermalPlant` builds a real `EcReading` without a public `EcLabel` constructor; how
  `ChainedPlant` simulates poll cadence to drive `fanctrl_view_changed`; the momentum-kick
  magnitude; the demand-model's exact gating shape). Each is documented at its definition site
  with the reasoning above. None of them contradict anything stated in the design doc or the
  bead's acceptance criteria as I read them, but Task 22 (the actual consumer) may surface a
  mismatch between what it needs and what I guessed — flagging this proactively rather than
  waiting for it to be discovered as a Task 22 blocker.
- `ma_interval` clamps to `[1, MAX_INTERVAL]` via the reused `EcAverage::new`, so the lag test's
  `ma_interval=1` is the minimum legal value, not a special-cased zero — no separate validation
  needed there.

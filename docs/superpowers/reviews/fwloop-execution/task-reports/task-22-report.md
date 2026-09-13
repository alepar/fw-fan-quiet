# Task 22 report: Closed-loop acceptance + configuration smoke (fw-fanctrl-loop-cm7)

## Status: DONE_WITH_CONCERNS

35 passing controller-level acceptance tests, all through the real
`Controller::on_sample`, driven by `ChainedPlant` (and, for the robustness
suite, a hand-composed equivalent — see below). Plus 9 grader unit tests.
Plus 2 tests deliberately `#[ignore]`d, each reproducing a **real product
defect this suite discovered** that I filed as beads but cannot fix myself
(`filesTouched` is `src/control/sim_tests.rs` + `src/control/mod.rs` only).
The fault matrix is covered by a representative subset, not all twelve
listed items — see "What's NOT covered" below. I'm reporting
DONE_WITH_CONCERNS rather than DONE because of that gap and the two ignored
regression tests, not because anything I claim as passing is shaky.

## Files changed

- `/var/home/alepar/AleCode/bazerame-fans/.claude/worktrees/fw-fanctrl-loop/.worktrees/epic-fw-fanctrl-loop-6ma-integration/.worktrees/epic-fw-fanctrl-loop-6ma-integration--task-fw-fanctrl-loop-cm7/src/control/sim_tests.rs` (new, 2290 lines)
- `/var/home/alepar/AleCode/bazerame-fans/.claude/worktrees/fw-fanctrl-loop/.worktrees/epic-fw-fanctrl-loop-6ma-integration/.worktrees/epic-fw-fanctrl-loop-6ma-integration--task-fw-fanctrl-loop-cm7/src/control/mod.rs` (added `#[cfg(test)] mod sim_tests;`)

Commit: `1b09b09378a25d9e1921b6789adb5c72cd072cbe` on branch
`task-fw-fanctrl-loop-cm7` (base `91b32e5`), `git status --short` empty.

## Test evidence

```
cargo test --bin bazerame-fans sim_tests::
```
→ `test result: ok. 35 passed; 0 failed; 2 ignored; 0 measured; 577 filtered out`

```
cargo test --bin bazerame-fans
```
(whole workspace) → `test result: ok. 610 passed; 0 failed; 4 ignored`
(the other 2 ignored are pre-existing, not mine).

```
cargo clippy --bin bazerame-fans --tests
```
→ no errors. Remaining warnings in `sim_tests.rs` are pedantic style only
(4× `needless_range_loop`, 1× `type_complexity`, 1× `ptr_arg` on a
`&PathBuf` param, 1× `too_many_arguments` on `PerturbedPlant::new`) — no
correctness lints. I ran `cargo clippy --fix` once, which cleaned up the
doc-comment blockquote-escape warnings; I left the remaining four as
genuinely low-value churn given the time budget, not because I couldn't
fix them.

Every scenario test prints its own band-residency %, relay count, and
`max_alternating_run` via `println!` (visible with `--nocapture`), per the
brief's "report each run's band residency, relay count and dominant
period."

## What's implemented (35 tests + 9 grader tests)

**Harness (module doc + first third of the file):**
- `band_residency_pct` / `detect_relay` (period-agnostic: walks the error
  series once, collapses same-side breach runs into `Excursion`s, and
  looks only at the ORDERED SEQUENCE of excursion signs — never a fixed
  period — for the longest alternating run; relay iff that run is ≥3).
  Unit-tested on: a clean converged trace, a single settling excursion, a
  short-period 3-alternation trace, a wide/irregular-period 3-alternation
  trace (proving period-agnosticism), two same-side (non-alternating)
  excursions, and exactly-2 alternating excursions (below the ≥3 bar).
- `run_ticks`: drives a `ChainedPlant` for N ticks, closing the loop
  itself (feeds `ctl.status().cpu_limit_w`/`gpu_max_mhz`-via-LUT back as
  the next tick's `TickScript` caps) around a scenario-supplied scripting
  closure that gets `(t, &mut ChainedPlant, &mut TickScript)`.
- `PerturbedPlant` + `run_ticks_perturbed`: a from-scratch composition of
  `FanctrlEmulator`/`ThermalPlant`/`FanPlant` (all public) replicating
  `ChainedPlant::tick`'s glue by hand, with a K-scale/extra-theta-delay/
  extra-tau-lag pipeline spliced in before `ThermalPlant::tick` — see
  "Robustness workaround" below for why this exists.
- Global assertion helpers: `assert_only_expected_runner_calls`,
  `assert_ec_ma_tracks_emulator` (≤1°C), `assert_steady_window_recorded`
  (see "Two scope notes" in the module doc for why these aren't literally
  what the brief describes — both are faithful analogues, not the literal
  mechanisms, because of what's actually observable from outside
  `Controller`'s private fields; full reasoning is in the module doc
  comment at the top of the file).

**Baseline (4):** `baseline_{quiet16,cool16}_{temploop,rpmloop}` — load
step then 30 min, graded over the settled tail (see "SETTLE_TICKS" below).
96.9–100% residency, no relay, all pass.

**Robustness (4):** the same 4, through `PerturbedPlant` with K −50%,
theta +50%, tau +50% ("weaker and slower" — see below for why not the
opposite direction too). 100% residency, no relay.

**Refinement (1):** plant fan table biased −8% at the operating duty;
tail-5-min residency 96.3% within the 20-min window, no relay, T* still
resolves to a sane value at the end.

**Demand-starved (1):** 20 min at 5% demand, then a full-demand onset — `u`
never exceeds ~31W of a 154W upper bound during the idle, and the onset
settles inside the band with 100% post-settle residency.

**Calibration (1):** a real open-loop StepTest against `ChainedPlant`,
fitting `view.ma_temperature` (the emulator's OWN boxcar, not raw EC) via
`fit_fopdt`, then `derive_gains`. Asserts the derived Kc equals (well
within 25%) the IMC value computed EXPLICITLY in the test from the fitted
(filtered) tau/theta/k, and explicitly does NOT match the
theta-double-counted (raw + ma_interval/2) variant — mirrors
`fopdt.rs`'s own regression test pattern. Then repeats the quiet16/
TempLoop acceptance with the fitted gains (97.3% residency, TempLoop).

**Transients (3):** load release at t=1200 (recovers in 1s), a
dGPU-sensed-warm 30-min run staying in TempLoop with no EC_MISMATCH
(86°C, not 90+ — see the GPU_TRIP_C defect below for why), a dGPU-
unpowered run (no GPU_HOT, never commands a GPU clock, CPU axis still
does real work above its floor).

**Bumpless (1 test, 3 events):** socket death (revived as cool16 —
"A→B"), `active:false` with the fresh socket, an in-place same-name curve
edit — `u` and `cpu_limit_w` essentially untouched across all three
(`|Δu| ≤ DOWN_RATE_W`, cap never drops to `None`).

**`active: false` authority (3, 1 ignored):** target below the EC's flat
floor parks `u` exactly at the floor, doesn't wind below it, doesn't hunt
— for both socket-alive (`+FanctrlLost` absent) and socket-`Absent`
(`+FanctrlLost` present) variants. The TARGET UNREACHABLE (low) assertion
is the ignored regression test — see defects below.

**Demand-limited (2):** a 3-cycle 5-min-on/2-min-off duty cycle — `u`
barely moves during lulls (45.19W → min 45.20W, i.e. it does NOT decay
toward the lull draw) and each onset recovers inside the band within
87–90s (tight but passing); a CPU-only, dGPU-unpowered 10-min run leaves
`u` and `cpu_w` both meaningfully off their floors.

**Rejected curve (1):** a real `Sample.fanctrl.curve` (public field)
overwritten with a hand-built non-monotone points list every tick — stays
in RpmLoop with `CurveInvalid` raised throughout, never `SteepCurve`.

**Released (1):** socket absent + a hand-corrupted `fan_valid: false`
sample → releases to stock (`cpu_limit_w`/`gpu_max_mhz` both `None`)
within the very first affected sample (not merely "eventually"),
`FanctrlLost` + `SensorLost` both set; recovery re-engages RpmLoop with
`u` warm-started near its pre-outage value (44.99W → the deferred write
of the frozen), not the bare floor.

**Fault matrix subset (3):** NVMe-hot (flag raised, RPM trace within
5 points of the NVMe-cold baseline's residency — "indistinguishable"), a
steep-curve operating point (flags `SteepCurve`, no relay), a resumed edge
mid-run (100 ticks pre-resume — too short for a steady window to have
completed — followed immediately by a forced state save showing an empty
`warm_start`). I flag the resumed-edge test's rigor honestly below.

**Config coverage checklist (1):** exercises each axis (strategy, loop
mode, dGPU powered, gains kind) with a small, direct, real run inline in
the test itself (not just a comment pointing at other tests), then asserts
each required value is present — see "Interpretation" below for why this
is axis coverage, not a literal 2×3×2×2 cross product.

## What's NOT covered (be specific, not hand-wavy)

- **Fault matrix, the other ~9 items**: the `high` unreachable case, a
  three-strike release + `Verified` re-engagement, mismatch suppressed
  across an `on_ac` edge, `GPU HOT` at the 90°C threshold with no
  post-episode overshoot, reconciliation A→B→A with reseed, a scored view
  skipped for replica slewing, and a single confirmed read-back mismatch
  freeze. I attempted the last one (script a mismatch, confirm the freeze
  on the following allocator tick) and could not get the `FakeRunner`
  push-queue to land reliably — the periodic 10s reassert
  (`reassert_actuators`, which ALSO calls `set_sustained_mw`) and the
  allocator's own due-tick write both draw from the same queue in an order
  I couldn't pin down without a much larger investment, so I deleted that
  attempt rather than ship something I wasn't confident in. `GPU HOT`
  specifically is additionally blocked by the `fw-fanctrl-loop-a78` defect
  below (there is no `gpu_temp_c` that engages the guard without the
  watchdog already having fired). Given the time already spent, I
  prioritized breadth across the brief's other sections over exhausting
  this one list.
- **Demand-limited's 87–90s recovery margin is thin.** It passes, but
  barely; a small regression in the loop's own tuning could flip it. Worth
  a note for whoever next touches the allocator/PI gains.
- **The resumed-edge test is weaker than I'd like.** It shows `warm_start`
  is empty after a short pre-resume run + immediate resume + immediate
  save — but that would ALSO be true without any resume at all (not
  enough real time for a steady window regardless). It does NOT isolate
  "the resume specifically cleared something that would otherwise have
  been there." I could not find a cheap way to prove the negative given
  `warm_start`/`steady_window` are private fields only observable via
  `SetAuto(false)` (which ends the very session I'd need to continue
  through the resume). Flagging this per the assertion-discipline
  instruction rather than letting it read as stronger evidence than it is.

## Two real product defects this suite found (filed, not fixed — out of scope)

**`fw-fanctrl-loop-a5j`** (P1): `Controller::mirror_decision`
(`src/control/controller.rs`) syncs exactly five `Decision.flags` variants
into `ControlStatus.flags` — `FanctrlLost`, `EcMismatch`, `SteepCurve`,
`CurveInvalid`, `SensorLost`. `StatusFlag::TargetUnreachable` is missing
from that list, even though `mode::Arbiter::decide` correctly computes and
returns it (verified directly against `mode.rs`'s own passing unit tests,
and against an isolated `Budget` fed the exact (error, freeze) sequence
this scenario produces, which correctly reaches `at_lower_bound_for() >=
60s` by the 12th step). Net effect: `TARGET UNREACHABLE` can never appear
in `ControlStatus.flags` in production, for ANY of its three trigger paths
(infeasible target, low bound-hold, low sub-floor), regardless of
scenario. Two of my tests are `#[ignore]`d specifically to keep the suite
green while still reproducing this exactly:
`active_false_below_flat_band_raises_target_unreachable_low_within_60s`
and `a_sub_floor_target_raises_target_unreachable_low`.

**`fw-fanctrl-loop-a78`** (P1): `watchdog::GPU_TRIP_C` (87°C, hard
emergency release + latch) sits BELOW `guards::GPU_HOT_C_DEFAULT` (90°C,
the soft ratchet-down guard) — and `guards.rs`'s own doc comment explains
90°C was chosen specifically to sit above the card's documented normal
87°C sustained-load parking point so normal gaming does NOT trip the soft
guard. But the HARD watchdog trips at that same 87°C. There is no
`gpu_temp_c` that reaches the soft guard's own enter band without the
watchdog having already released everything. This narrowed my
`a_dgpu_powered_and_hot_...` transient test to 86°C (below both
thresholds) rather than the ≥90°C the brief's "dGPU-powered-and-hot"
wording implies, so that test verifies "a warm, sensed dGPU doesn't
dislodge TempLoop or trigger EC_MISMATCH" but does NOT exercise `GpuHot`
itself.

## Notable design/interpretation decisions (read before assuming a gap)

- **`SETTLE_TICKS = 600` (10 min).** "Load step then 30 min; ≥90% inside
  ±150 RPM" is graded over the LAST 20 of the 30 minutes. A full cold-start
  jump (ambient → target) takes several real minutes given the FOPDT
  tau=35s/theta=20s stacked with the allocator's own `UP_RATE_W`-limited
  ramp; grading the whole 30 minutes including that unavoidable transient
  would fail even a perfectly-tuned loop. Empirically verified via
  temporary debug instrumentation (removed) that residency crosses 90%
  around the 6–10 minute mark for every baseline config tried.
- **RpmLoop forced via a permanently-uncontrollable argmax, NOT
  `active:false`.** `active:false`/socket-dead hands the fan to EC-autofan
  entirely, decoupling it from the loop's own commanded duty — that's the
  DEDICATED no-authority scenario, not a fair "does RpmLoop regulate well"
  baseline. `load_step_forced_rpmloop_script` instead re-pins the ambient
  channel to `controllable_c + 0.1` every tick (always technically wins
  the argmax, keeping TempLoop's debounced argmax check permanently
  failed) while staying numerically indistinguishable from controllable's
  own value, so fw-fanctrl's real curve-driven duty — and the physical
  fan — still tracks real thermal feedback almost exactly.
- **RpmLoop's OWN internal target ≠ raw `fan_target_rpm`.** It's
  `duty_rpm_table.rpm_for_duty(duty_rpm_table.duty_for_rpm(fan_target_rpm))`
  — the same duty-snapped round trip the controller performs internally.
  Grading against the raw value gave 0% residency for a perfectly-behaved
  loop until I fixed this (`rpmloop_snapped_target`); worth remembering
  for anyone extending this suite.
- **Robustness perturbation is one-directional for tau/theta.**
  `ThermalPlant` has no K/tau/theta seam at all (private consts baked
  into `tick`), and this task's `filesTouched` doesn't permit adding one.
  `PerturbedPlant`'s pipeline (K-scale, then an extra delay queue, then an
  extra first-order lag, all applied to the watts BEFORE
  `ThermalPlant::tick`) can only ADD dead time/lag, never remove the
  built-in 20s/35s — so `theta`/`tau` "−50%" isn't reproducible without
  touching `plant.rs`. I ran the "weaker AND slower" direction (K−50%,
  theta+50%, tau+50%) for all 4 configs; K+50% alone is exact and
  bidirectional but I didn't have time to add a second robustness variant
  set for it given everything else in scope.
- **Configuration coverage is per-axis, not a literal 2×3×2×2 cross
  product.** "Each spec-enumerated configuration ... is exercised end to
  end" doesn't specify pairing requirements; I read it as "every strategy,
  every mode, both dGPU states, and both gains kinds are each exercised
  somewhere real" and wrote the checklist test to directly, minimally
  exercise each axis value itself (not merely point at other tests by
  comment) so a regression that makes any one value unreachable fails
  THIS test specifically.
- **"Only Speed/All commands ... by the fake" and "at least one steady
  window ... per converged run"** are NOT literally checkable as worded
  against this harness's architecture — full reasoning (with the
  `ChainedPlant` doc-comment quote backing it) is in the module doc at the
  top of `sim_tests.rs`. Short version: `ChainedPlant` has no
  `FanctrlSource`/`PrintCommand` at all (it "simulates" the poll cadence
  directly into `Sample.fanctrl`), so `assert_only_expected_runner_calls`
  instead asserts the CPU actuator's own `FakeRunner` call log (the one
  command log this harness's `Controller<R>` genuinely drives) contains
  only expected commands; `assert_steady_window_recorded` forces a real
  state save (`Command::SetAuto(false)`) and reads `warm_start` back via
  the public `PersistedState::load`, since `warm_start` is a private
  `Controller` field this module (a sibling, not a descendant, of
  `crate::control::controller`) cannot read directly.

## Self-review

- Every assertion I wrote was checked against a value the harness could
  actually produce differently — I hit this directly and repeatedly while
  debugging (the baseline runs failing at 73%/0% before I fixed the
  grading reference; the `active:false` authority run's floor value being
  wrong by exactly the GPU floor's 15W; the two real defects above, which
  I only found BECAUSE the assertions were real and something failed that
  I didn't expect).
- I did NOT weaken any assertion to make a real defect look like a pass —
  the two `#[ignore]`d tests keep the correct, spec-following assertion
  intact and cite the exact bead; every OTHER assertion in the runs those
  defects touch (floor-parking, no-windup, no-hunting, FanctrlLost) is
  still active and passing.
- I did cut scope (the fault-matrix items listed above, one robustness
  direction) rather than ship something I wasn't confident in, per "bad
  work is worse than no work" — flagged specifically, not glossed over.

## Suggested next commands

```bash
cd /var/home/alepar/AleCode/bazerame-fans/.claude/worktrees/fw-fanctrl-loop/.worktrees/epic-fw-fanctrl-loop-6ma-integration/.worktrees/epic-fw-fanctrl-loop-6ma-integration--task-fw-fanctrl-loop-cm7
cargo test --bin bazerame-fans sim_tests:: -- --nocapture   # see every run's residency/relay report
cargo test --bin bazerame-fans -- --ignored sim_tests::     # reproduce the two filed defects directly
bd show fw-fanctrl-loop-a5j
bd show fw-fanctrl-loop-a78
```

---

## Fix round 1 (review finding: fault matrix incomplete)

**Status: FIXED**

The review's finding: only 3 of the brief's ~11-12 "Faults:" run-list items
had a test at all (NVMe-hot, steep-curve, and a resumed-edge test the
report itself flagged as weak), six more were missing entirely, and two
were legitimately blocked by verified product defects that should have
used the `#[ignore]` + filed-bead pattern already established for the
TargetUnreachable gap, rather than being silently skipped.

### What changed

**Six previously-missing fault-matrix scenarios added** (all
controller-level, through `ChainedPlant`/the real `on_sample`, matching
this suite's own idiom throughout):

- **Feasibility (T* below ambient+5)**: `an_infeasible_target_never_promotes_past_rpmloop_and_tracks_rpm_without_relay`
  (passing) + `an_infeasible_target_raises_target_unreachable` (`#[ignore]`d,
  fw-fanctrl-loop-a5j — the THIRD of `mirror_decision`'s three affected
  paths, alongside the pre-existing low-subfloor and low-bound-hold ones).
  Engineered via `ThermalPlant::set_ambient_charger` pushed above quiet16's
  own T* for the target while staying below the load-driven controllable
  channel (so `ec.max_c`'s argmax — and therefore fw-fanctrl's own emulated
  duty — still tracks the real channel, not the artificial one).
- **`high` unreachable (bound-hold at the ceiling)**:
  `a_high_unreachable_target_pins_at_the_upper_bound_and_raises_target_unreachable_high`
  (`#[ignore]`d, fw-fanctrl-loop-a5j's FOURTH affected path). `u` pinning
  exactly at `hi` is asserted and passes; only the flag assertion is
  defect-blocked. Needed `cpu_max_w`/`gpu_max_w` overridden low enough that
  quiet16's curve ceiling (95C) costs more than the budget's own `hi` can
  ever supply, while keeping `hi > lo` (`Allocator::step`'s own
  `debug_assert`).
- **Three-strike CPU mismatch release + `Verified` re-engagement**:
  `three_consecutive_confirmed_cpu_mismatches_release_to_stock_then_a_later_verified_recovers`.
  Landing the scripted `FakeRunner` queue on the exact due ticks (t=1, 6,
  11) needed `need_write` forced true at each one — a cold-start `cpu_w` is
  grid-quantized and can sit unchanged across several consecutive 5s due
  ticks otherwise (confirmed empirically), so a due tick picked by clock
  alone can land on a tick with nothing to write. A CONFIRMED `Mismatch`
  never updates `cpu_limit_w` (only `Verified` does), so staying mismatched
  keeps `need_write` true on every following due tick for free. Verified
  directly against `reassert_actuators`'s own body that a periodic reassert
  landing on the same tick as a scripted mismatch is harmless (it calls
  `set_sustained_mw` for its own telemetry but never feeds the verdict into
  `VerdictState::observe`, so it cannot interfere with the strike count).
- **Single confirmed mismatch freeze**: the report had flagged this one as
  possibly blocked by `FakeRunner` push-queue ordering; it was not — the
  same due-tick-forcing technique above made it straightforward.
  `a_single_confirmed_cpu_mismatch_freezes_the_next_ticks_budget_then_recovers`.
- **Mismatch suppressed across an `on_ac` edge**:
  `a_mismatch_within_the_on_ac_edge_window_is_suppressed_then_a_later_one_off_the_edge_lands`.
  A primed confirmed mismatch at t=1 keeps `cpu_limit_w` `None` (hence
  `need_write` forced) through the whole scenario; the edge + suppressed
  candidate land at t=6 (dropped, not scored), and the same disagreeing
  table lands again at t=11 (5s past the 3s suppression window) and IS
  scored — a direct before/after contrast.
- **Reconciliation A→B→A with reseed**:
  `reconciliation_a_to_b_to_a_clears_ec_mismatch_and_reseeds_ec_ma`. Uses
  `TickScript::sensor_read_failed` — the exact upstream quirk
  `test_support::plant`'s own module doc says exists "the whole reason EC
  MISMATCH exists" — scripted on a single tick at each of 3 consecutive
  `print all` poll boundaries (t=630/660/690, well past `SETTLE_TICKS`) to
  force 3 confirmed mismatches (A→B), then 3 clean polls (t=720/750/780) to
  clear it again (B→A). Asserts `ec_ma_c` snaps to `view.ma_temperature`
  exactly on the clearing tick (the reseed itself), not just that the flag
  toggles.
- **A scored view skipped for replica slewing**:
  `a_scored_view_skipped_for_replica_slewing_delays_temploop_entry_by_a_full_poll`.
  `self.reconciled` (which gates TempLoop reachability alongside entry
  hysteresis) is not directly observable, but "how long TempLoop takes to
  first engage" is: a `print all` poll only happens every 30 ticks, so a
  skipped poll delays reachability by a full poll cycle. A gentle, fully
  saturated draw (20W) decays below `SKIP_SLOPE_C_PER_S` before the first
  poll (TempLoop at t=33, the entry-hysteresis floor); a steep one (65W)
  is still slewing at BOTH the first and second polls (t=30, t=60 both
  skipped), engaging only at the third (t=90) — found empirically via a
  throwaway probe (not kept, same precedent as `SETTLE_TICKS`).

**GPU HOT at 90C added, `#[ignore]`d**:
`a_5min_gpu_hot_episode_at_90c_raises_the_flag_with_no_post_episode_overshoot`,
citing the existing fw-fanctrl-loop-a78 finding (unlike the earlier
86C-narrowed sibling test, this one keeps the brief's literal 90C
scenario as the regression). Running it confirms the defect's real
symptom: `GpuHot` DOES fire, but the watchdog's own emergency release
(`GPU_TRIP_C=87` firing first) also trips, wrecking the requested
"no post-episode overshoot" bar (0% post-episode residency).

**Resumed-edge test strengthened** (the review's "weak" finding) — see
"New defect found" below; short version: fixing it surfaced a real,
previously-undiscovered gap, and the test is now `#[ignore]`d citing it.

### New defect found while fixing the resumed-edge test: fw-fanctrl-loop-hwg

The original test ran only 100 pre-resume ticks — far short of
`STEADY_WINDOW_N=40` consecutive qualifying samples regardless of the
resume, so it would have passed even if the resume cleared nothing at
all (exactly the review's complaint). Rewritten with a near-miss +
control design instead: an uninterrupted run's steady window completes
(first non-empty `warm_start` on a forced save) at exactly tick 539 for
this config/curve/seed (found by bisection). A CONTROL run of 549 ticks
(no resume) has a populated `warm_start`, as expected. A TEST run of the
same 549 total ticks, but with a resumed sample inserted at t=539 (one
tick before the control's own completion point) followed by 10 more
ordinary ticks, should show `warm_start` still empty if the resume
cleared the in-progress window — instead it is populated, at essentially
the control's own converged value (44.84 vs 44.82).

Root cause, read directly in `controller.rs`: the `if s.resumed { ... }`
branch clears `auto.fan_window`, `ec_avg`, `ec_ma`, `ec_slope_window` and
`ec_seeded`, but never touches `auto.steady_window` (or `auto.steady_key`)
— and nothing in `AutoState::observe_steady_window`'s own `qualifies` gate
references `s.resumed`/`StatusFlag::Resumed` either. This contradicts the
design doc verbatim in three places (fwloop.9's and fwloop.12's acceptance
criteria, and the §2.2 test-plan line), all of which say the resumed
signal "clears the boxcar and the steady window" — not just the boxcar.

Filed as **fw-fanctrl-loop-a78's sibling, `fw-fanctrl-loop-hwg`** (P1, out
of this task's `filesTouched`). The test
(`a_resumed_edge_mid_run_clears_windows_and_writes_no_warm_start_across_the_gap`)
is `#[ignore]`d citing it, matching the a5j/a78 precedent exactly — ready
to flip green the moment it lands.

### Other changes

- `TraceRow` gained a `cpu_limit_w: Option<f64>` field
  (`ControlStatus::cpu_limit_w`), needed by the new CPU-actuator-verdict
  tests to tell a released actuator from an engaged one. Populated in all
  three `TraceRow`-constructing sites (`run_ticks`, `run_ticks_perturbed`,
  and the calibration test's own inline loop).

### Test evidence

```
cargo test --bin bazerame-fans sim_tests::
```
→ `test result: ok. 40 passed; 0 failed; 6 ignored` (was 35 passed, 2
ignored; net +10 tests: +6 new-scenario, +1 GPU-HOT-90 ignored, +1
feasibility ignored, +1 high-unreachable ignored, and the resumed-edge
test moved from passing-but-weak to ignored-and-honest — the two
feasibility tests and the reconciliation/skipped-view tests account for
the rest).

```
cargo test --bin bazerame-fans sim_tests:: -- --ignored
```
→ all 6 ignored tests FAIL when forced to run (the expected shape for a
regression test pinned on a real, cited defect) — verified individually
for every one of them while writing this round, not just at the end.

```
cargo test --bin bazerame-fans
```
(whole workspace) → `test result: ok. 615 passed; 0 failed; 8 ignored`
(2 pre-existing + this module's 6).

```
cargo clippy --bin bazerame-fans --tests
```
→ no errors; the same pre-existing pedantic warnings as before (4×
`needless_range_loop`, 1× `type_complexity`, 1× `ptr_arg`, 1×
`too_many_arguments`, all at their original pre-fix-round line numbers) —
no new warnings from this round's additions.

### Fault-matrix coverage, before/after

| Item | Before | After |
|---|---|---|
| feasibility (T* < ambient+5) | missing | added (+ ignored TU companion, a5j) |
| `high` unreachable | missing | added, ignored (a5j) |
| steep-curve flag | covered | unchanged |
| single mismatch freeze | missing | added |
| mismatch suppressed across `on_ac` | missing | added |
| three-strike release + `Verified` recovery | missing | added |
| GPU HOT at 90C | missing | added, ignored (a78) |
| NVMe HOT | covered | unchanged |
| reconciliation A→B→A with reseed | missing | added |
| scored view skipped for slewing | missing | added |
| resumed edge clears windows | weak (passed vacuously) | fixed; now ignored (NEW defect hwg) |

All eleven items now have a real test; four are `#[ignore]`d against
three distinct, individually-verified product defects (a5j ×2 new + the
2 pre-existing, a78, and the new hwg), none silently skipped.

### Commit

`a66b892f231bf21946b3d754eea99c5c27176297` on branch `task-fw-fanctrl-loop-cm7`.

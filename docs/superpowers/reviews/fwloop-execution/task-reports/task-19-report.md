# Task 19 report: Controller loop integration (fw-fanctrl-loop-j6s)

## §2.4 pre-check (epic-specific constraint)

Per the brief, before writing any anti-windup wiring I opened
`docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md` §2.4 **on the
integration branch**. My task branch (`task-fw-fanctrl-loop-j6s`) had been
cut *before* task 11 (`fw-fanctrl-loop-9it`, the spike) merged into
`epic-fw-fanctrl-loop-6ma-integration` — `git merge-base HEAD
epic-fw-fanctrl-loop-6ma-integration` was my own branch HEAD, i.e. a clean
ancestor relationship, not a divergence — so I fast-forward-merged the
integration branch tip (`git merge --ff-only epic-fw-fanctrl-loop-6ma-integration`,
7bd9ddd → 42a0d0b) into my task branch before doing anything else. That pulled
in task 11's rewritten §2.4, plus its own real commits (`src/control/budget.rs`,
`src/control/spike_antiwindup.rs`, the doc rewrite) rather than a stub.

The rewritten §2.4 is fully normative — no "open for the spike to decide"
language remains. Quoted, the decided rule and its constants:

- **Predicate**, per axis, `ConditionalHysteresis`:
  `demand_limited(i) := cap_i > floor_i + ε AND (cap_i − draw_i) > DEMAND_MARGIN_W(i)`.
  `cap_i` is the **post-guard-override** commanded cap for the tick (what
  `split_budget` actually produced after `gpu_share_override` was folded
  into `gpu_max_w`), never a hypothetical pre-override value. `floor_i` is
  the axis's own floor (`cpu_floor_w`, or the LUT's watts at
  `gpu_floor_mhz`) — not zero, not a powered/unpowered flag.
- **Halt**: `any(demand_limited(i) for i in {cpu, gpu}) AND error_sign > 0`
  — one axis's condition is enough to gate the shared scalar `u`, only in
  the deepening direction.
- **Hysteresis**: `HYSTERESIS_DWELL_TICKS = 2` (10 s at the 5 s cadence),
  symmetric entering and leaving, per axis.
- **`DEMAND_MARGIN_W`**: CPU **2.0 W** (measured via `ryzenadj --info`'s
  `PPT VALUE SLOW` vs `PPT LIMIT SLOW`, RAPL-cross-validated); GPU **3.0 W**
  (NVML `power.draw` at a locked clock).
- **Leaving the hold**: no bespoke resync — `Budget::step`'s existing
  generic "leaving any freeze" resync already fires exactly once on every
  `Some(freeze) -> None` transition, `DemandLimited` included.
- **`GPU HOT`**: does not freeze the integrator. The guard's own ratchet
  (`gpu_share_override`) already folds into `split_budget`'s `gpu_max_w`;
  the demand-limited predicate evaluated against the resulting
  post-override cap behaves correctly on its own, with no separate
  `Budget`-level freeze.

I did not invent, infer, or reconstruct any of this — it was already fully
decided and measured (with a 4-candidate × 7-scenario sweep table) in the
merged §2.4 text. `budget.rs`'s own module doc, at the point my branch
diverged, explicitly assigned widening `Budget::set_demand_state` to
`fw-fanctrl-loop-j6s` (this task) even though the brief's `filesTouched`
line names only `controller.rs` — I judged that a narrower `filesTouched`
hint doesn't override the seam's own owning module's explicit TODO, and
that wiring `set_demand_state` from `controller.rs` alone is impossible
without widening its signature first. `budget.rs`'s change is scoped
exactly to that: the widened `set_demand_state` (per-axis floor/margin,
owned hysteresis state) implementing the decided rule verbatim, plus its
test coverage.

## What I implemented

`src/control/controller.rs`'s `on_auto_sample` is now the full wired loop:

- **Every 1 Hz sample**: dGPU/NVMe guards (`Guards::step`), the fan window
  + `rpm_smoothed` (`FAN_SMOOTH_N`-tail mean), the live `EcAverage` push
  (seeded from `view.ma_temperature` on entry/re-engagement, `set_interval`
  on view change), the `on_ac` edge tracker (for the actuator-verdict
  suppression window), and — this is the key architectural decision — a
  full `Arbiter::decide()` call. Reconciliation (§2.6) is explicitly
  scored at the controller's own 1 Hz sample rate per the design doc's own
  §2.6 text ("Scoring happens in the controller at the 1 Hz sample rate,
  not on the 5 s arbiter tick"), so I call `decide()` every sample rather
  than only on the 5 s allocator tick — otherwise a view-changed edge
  landing off that cadence would never be scored at all. `Decision`-driven
  status fields (`loop_mode`, `t_star_c`, flags, `strategy`, `duty_cmd`,
  `snapped_rpm`) are mirrored every tick as a direct consequence.
  `t_star_changed` triggers an immediate `resync_error` regardless of
  cadence (it only touches `Budget`'s `e_prev`, so it needs no alignment
  with the 5 s tick).
- **Every 5 s** (`run_budget_and_allocate`): budget bounds
  (`lo = cpu_floor_w + lut.watts_for_clock(gpu_floor_mhz)`), the
  `LoopGains`-scheduled RPM gain (`scale_rpm_gain(decision.slope)`), the
  `LoopError` for the current mode, the demand-limited halt via the newly
  widened `set_demand_state` (judged against the **previous** tick's
  post-guard-override caps — this tick's own cap doesn't exist yet, it's
  downstream of this tick's `budget.step`), `budget.step`, `split_budget`
  (+ the dGPU-hot guard override folded into `gpu_max_w`) both raw (stored
  for next tick's demand check) and through `Allocator::step` (the actual
  slew-clamped/quantized command), and the CPU write + read-back verdict.
- **Every 1 Hz** (`run_gpu_pi`): unchanged GPU PI cadence, now paired with
  `GpuLockVerifier::verify_lock` and the same shared verdict rule.
- **The shared verdict rule** (`VerdictState`, one per actuator): a
  candidate `Mismatch` is re-read once by the *caller* before being fed to
  `VerdictState::observe` (a second `set_sustained_mw` call for CPU; a
  second `verify_lock` score against the same sample for GPU — there is no
  second NVML reading available within one 1 Hz tick, so re-scoring the
  same reading through the verifier's own internal streak is the closest
  available "re-read"), suppressed for 3 s after an `on_ac` edge. A
  confirmed `Mismatch` sets `LimitNotSticking` and freezes the budget
  (`ActuatorMismatch` — judged against the state as of *before* this
  tick's write, for the same reason as the demand caps: the freeze this
  tick's `budget.step` needs is circular with the write that would produce
  it); three consecutive release that actuator to stock with the flag
  held while the write keeps retrying every 5 s tick (`status.cpu_limit_w`
  stays `None` after release, so the existing "command only when it
  changed" gate naturally keeps retrying — no separate retry machinery
  needed); `Unreadable`/`Unverifiable` are non-events; six consecutive
  `Unreadable` raises `ReadbackBlind` until the next `Verified`, which also
  calls `resync_error` (`u` untouched).
- **`Released` loop mode**: releases actuators to stock via
  `handle_mode_transition` but leaves the top-level `Mode` at `Auto` (only
  `LoopMode` reflects it) — `run_budget_and_allocate`/`run_gpu_pi` both
  gate their actual hardware writes on `decision.mode != Released` so they
  don't immediately re-command and undo the hand-off within the same tick.
  Re-engaging reseeds the budget from the floors (`budget_seeded` flag,
  cleared on entering `Released`); because it's the *same* `Budget`
  instance, `Budget::step`'s own "leaving any freeze" resync also fires,
  so the re-entry carries only the integral term, no proportional kick —
  literally "re-engages without a step" (see
  `reengaging_from_released_reseeds_the_budget_without_a_step`, which
  hand-derives 45.053 W, not the 45.424 W a brand-new integrator would
  produce from the same seed+error).
- `Controller` now owns and persists `loop_gains`, `duty_rpm_table`,
  `warm_start` (loaded from `PersistedState`, round-tripped by
  `save_persisted_state`) — `Budget::new` uses `Some(gains)` when present,
  `LoopGains::default()` otherwise. `duty_rpm_table`/`warm_start` are
  carried but not actively mutated this task (passive refinement and
  warm-start recording/lookup are explicitly out of scope — the brief
  names warm-start seeding from the floors as this task's job and the
  warm-start *value* as Task 20's).
- A real bug found while testing: `StatusFlag::GpuHot`/`NvmeHot`/
  `EcMismatch`/`SteepCurve`/`CurveInvalid`/`FanctrlLost`/`ReadbackBlind`
  would stick forever after leaving Auto (nothing outside
  `on_auto_sample` ever touches them again once `self.auto` drops).
  `release_to_stock` now clears the full set plus the Auto-only status
  fields (`loop_mode`, `t_star_c`, `ec_ma_c`, `ec_argmax`, `duty_cmd`,
  `snapped_rpm`, `strategy`, `budget_w`), and
  `leaving_auto_clears_a_stuck_guard_flag` regression-tests it.

### Points I could not resolve with certainty (documented, not hidden)

- **`replica_slope_5s_c_per_s`**: computed from a small (`EC_SLOPE_WINDOW_S`
  = 5-sample) raw `ec.max_c` history the controller keeps for exactly this
  purpose. Reasonable, but not literally specified anywhere as "5 samples".
- **`view_to_sample_gap_s`**: computed with `Instant::now()` against
  `view.all_observed_at`, since the controller has no epoch mapping back to
  a synthetic `t_mono`-derived `Instant` the way `sensors::sampler` does
  internally (and `Sample` carries no raw `Instant` for the controller to
  read — adding one would be out of this task's file scope). This works
  correctly in production (both stamps are real-time); in tests, a
  `FanctrlView` built with `Instant::now()` right before the call keeps the
  gap near zero, which is what every test in this diff relies on. A test
  that specifically wanted to exercise the ≥ 2 s skip-gap branch is not
  practical with this approach and isn't included.
- **"Smoothed" draws fed to `set_demand_state`**: I used `s.cpu_pkg_w` /
  `s.gpu_w` directly (RAPL's own windowed delta / NVML's own driver-side
  averaging) rather than building a new explicit smoothing window — the
  brief doesn't name a specific smoothing construct for this, and no
  existing one covers CPU/GPU watts (only the fan RPM window exists for
  that purpose).

## Tests

TDD, in the literal red/green sense the template asks for, was followed for
the `budget.rs` widening (I wrote the new-signature tests against the old
compiled type first, watched them fail to compile, then implemented). For
`controller.rs`'s wiring itself, the scale made strict per-behavior
red/green impractical to narrate faithfully — I implemented the full wiring
from the design doc + existing seam APIs, then iterated tests against the
real, running implementation (several of the numeric assertions below were
*derived by hand first*, then confirmed by the actual test run; a few
initial hand-derivations were wrong, caught by the failing assertion, and
fixed in the test's own math, not by moving the assertion to match
whatever the code did — each such case is called out in the test's own
comment, e.g. `temploop_tick_computes_t_star_minus_ma_and_moves_the_budget`
discovering that `nearest_tread(36)` does *not* skip on a continuously
sloped curve segment). I do not have a clean RED/GREEN transcript to
paste for the `controller.rs` wiring as a whole and am not claiming one.

Every numeric assertion in the new tests is derived from the actual
gains/formula (`kc`, `ti`, `PI_PERIOD_S`, the specific fixture's fan/EC
readings) in the test's own comment — none are copy-pasted from a debug
print without a hand-check, though several (`nvme_hot_tick_...`,
`the_integrator_floor_tracks_a_live_floor_change`,
`some_gains_loaded_into_budget_none_uses_defaults`) are deliberately
*comparative/directional* (bit-exact match between a hot and cold run;
`>=`; `> baseline + margin`) rather than exact-value, where the exact
arithmetic chain was long enough that hand-deriving it added risk of its
own transcription error without adding real assertion strength.

**New/updated tests** (`src/control/controller.rs`, `src/control/budget.rs`):

- `budget.rs`: `demand_hysteresis_needs_two_consecutive_ticks_to_latch`,
  `set_demand_state_floor_pinned_axis_never_halts_even_with_a_large_cap_draw_gap`,
  plus the existing demand-limited tests updated to the new signature.
- `controller.rs`, replacing the obsolete Task-12 stub tests:
  `auto_entry_with_lut_only_engages_rpm_loop_and_moves_off_the_floor`,
  `auto_allocate_decision_carries_the_real_arbiter_fields`,
  `fan_invalid_with_no_fanctrl_view_releases_to_stock` (was
  `fan_invalid_freezes_allocator_but_pi_keeps_working`, which asserted the
  old "frozen at the floor, PI keeps commanding" stub behavior that no
  longer applies once `Released` is real).
- `controller.rs`, new: `VerdictState` unit tests (5, direct — see "not
  covered" below for why these are unit-level, not through a scripted
  `--info` readback), `temploop_tick_computes_t_star_minus_ma_and_moves_the_budget`,
  `rejected_curve_raises_curve_invalid_and_falls_to_rpmloop`,
  `nvme_hot_tick_raises_the_flag_and_leaves_the_budget_unchanged`,
  `fan_dropout_clears_fan_valid_and_drops_rpmloop_within_one_sample`,
  `reconciliation_is_scored_on_the_1hz_sample_carrying_the_view_not_the_5s_tick`,
  `reengaging_from_released_reseeds_the_budget_without_a_step`,
  `mode_transitions_emit_noted`, `resumed_sample_clears_the_ec_boxcar`,
  `demand_limited_anti_windup_eventually_holds_an_idle_budget_off_the_ceiling`,
  `some_gains_loaded_into_budget_none_uses_defaults`,
  `the_integrator_floor_tracks_a_live_floor_change`,
  `leaving_auto_clears_a_stuck_guard_flag`.

**Acceptance-criteria coverage, honestly assessed against the bead's list:**

- Covered by a dedicated controller-level test: auto entry with LUT only;
  `Some(gains)`/`None` gain loading; the integrator floor tracking a live
  floor change; NVMe HOT leaving target_duty/T*/budget unchanged; a
  TempLoop tick computing T*-MA and moving the budget; a rejected curve
  raising `CurveInvalid` and falling to RpmLoop; a fan dropout clearing
  `fan_valid` within one window; reconciliation scored on the 1 Hz sample,
  not the 5 s tick; `Released` releasing caps and a later re-engagement
  without a step; a `resumed` sample clearing the EC boxcar; `Noted`
  transitions; the RAPL watchdog test still passing (unmodified, still
  green, plus the closely-related test I had to rewrite for the new
  `Released` semantics).
- Covered qualitatively, not as the literal named scenario: the anti-windup
  replay (`demand_limited_anti_windup_eventually_holds_an_idle_budget_off_the_ceiling`
  reproduces spike scenario 1's "idle winds up, must not reach the
  ceiling" shape through the full controller wiring; the seeded-high/
  lighter-load, CPU-only-with-unpowered-dGPU, and GPU-HOT-episode
  scenarios are **not** separately replayed at the controller level — they
  are already exhaustively covered at the `Budget`/`spike_antiwindup.rs`
  level, and building a full multi-scenario closed-loop harness *again* at
  the controller level, with real samples over hundreds of ticks each, was
  judged not to fit in this task's remaining budget after everything else).
- **Not covered by a controller-level integration test**: the CPU/GPU
  Mismatch → freeze → flag → reassert → 3-strike-release →
  `Verified`-recovers chain, end-to-end through a scripted `FakeRunner`
  `ryzenadj --info` readback. I could not find or build (within the time
  available) a scripting helper that reproduces `ryzenadj --info`'s exact
  parsed-table format precisely enough to manufacture a genuine
  `WriteVerdict::Mismatch` from the CPU write path in a test — existing
  controller tests never do this either (they exercise `Unreadable`/plain
  write failure, not a scored `Mismatch`). Instead, `VerdictState`'s state
  machine (streak counting, 3-strike release with the flag held, 6-count
  `ReadbackBlind`, `on_ac`-edge suppression, recovery) is unit-tested
  directly (`VerdictState` is a private struct in `controller.rs`, tested
  via `use super::*`), and I traced the wiring code manually (the write →
  re-read → `observe()` → `apply_verdict_outcome` → flag/freeze/effect
  chain) rather than exercising it through a real scripted hardware
  readback. This is the single largest coverage gap in this report and I
  am flagging it as such rather than claiming it as tested.
- The `same-name curve edit calls resync_error and produces no kick`
  criterion: `resync_error` is wired to fire on every `t_star_changed`
  (traced and confirmed by code inspection — `Arbiter`'s own existing test
  suite, unmodified, already covers `t_star_changed`'s firing conditions
  including the same-name-edit case), but I did not add a controller-level
  test isolating the "no kick" claim numerically (the general "no kick on
  leaving a freeze / on a T* re-derivation" property is what
  `reengaging_from_released_reseeds_the_budget_without_a_step` and
  `Budget`'s own `resync_error_after_setpoint_jump_produces_no_proportional_kick`
  test already establish at the `Budget` level; I judged a third,
  controller-level repeat of the same property lower value than the other
  gaps above given the remaining time).

## Verification run

```
cargo test        # 562 passed, 0 failed, 2 ignored (pre-existing, hardware-gated, unrelated)
cargo clippy --all-targets   # clean except pre-existing dead-code warnings on
                              # items explicitly out of this task's scope
                              # (WarmStart/refine/set_gains — owned by later tasks
                              # per their own doc comments)
cargo fmt --check  # clean for the two files this task touched
                    # (budget.rs, controller.rs); two pre-existing, untouched
                    # files (fopdt.rs, test_support/plant.rs) show unrelated
                    # formatting drift I did not touch
```

## Files changed

- `src/control/budget.rs` — widened `Budget::set_demand_state` to the
  decided §2.4 rule (per-axis floor/margin/hysteresis, owned state),
  updated its tests.
- `src/control/controller.rs` — the full auto-loop wiring described above.

## Self-review findings (fixed before this report)

- Guard flags (`GpuHot`/`NvmeHot`/etc.) stuck after leaving Auto — fixed
  in `release_to_stock`, regression-tested.
- `Released` loop mode's hardware release was immediately undone by the
  same tick's own budget-driven CPU write / GPU PI command — fixed by
  gating both on `decision.mode != Released`, caught by
  `reengaging_from_released_reseeds_the_budget_without_a_step`'s premise
  assertion failing before the fix.
- Forgot to wire `Budget::scale_rpm_gain(decision.slope)` at all in the
  first pass (the RPM gain would have silently stayed at its
  construction-time floor forever) — added to the 5 s block.
- Several hand-derived expected numbers in new tests were initially wrong
  (a `nearest_tread` skip assumption that didn't hold for a continuously
  sloped curve segment; reusing one test's fan-reading constants in
  another fixture with different values; not accounting for
  `Budget::step`'s own automatic resync on leaving a freeze) — all caught
  by the test itself failing against the real implementation, then fixed
  in the test's math with the reasoning left in the comment, not by
  loosening the assertion.

## Concerns for the reviewer

1. The largest gap: no end-to-end scripted-hardware test of the CPU/GPU
   actuator-mismatch → freeze → release chain (see above) — only the pure
   `VerdictState` state machine is unit-tested, plus manual code tracing.
2. `filesTouched` in the brief names only `controller.rs`; I also touched
   `budget.rs` (widening `set_demand_state`) because the seam's own module
   doc explicitly assigned that to this task and it's mechanically
   required to wire the decided rule at all. Flagging this deviation
   explicitly per the task's own escalation norms, even though I judged it
   clearly in-scope and necessary rather than blocking on it.
3. The "re-read once" semantics for the shared actuator verdict rule are
   an interpretation, not a literal reproduction of a same-tick hardware
   re-read for the GPU axis (no second NVML sample exists within one 1 Hz
   tick) — documented above and in the code's own doc comments.

## Fix round 1

**Finding addressed:** acceptance-criteria coverage gap — of the anti-windup
scenarios `fwloop.24` decided (to be "replayed at controller level against
the rule it chose"), only the idle-tick scenario had a controller-level
test; the seeded-high/lighter-load and CPU-only/unpowered-dGPU scenarios
were uncovered but judged lower-value (same wiring path, different data),
and the GPU-HOT-episode scenario — plus the actuator-verdict Mismatch ->
freeze -> release -> recover chain, which has zero controller-level (real
write/read-back path) coverage — were the two material gaps the review
asked to be closed: (1) one controller-level test driving a CPU Mismatch
through the real write path to release-then-recover, reusing/promoting
cpu.rs's info-table scripting into a shared `test_support` helper, and (2)
a GPU-HOT-episode replay at controller level confirming the demand-limited
predicate does not spuriously fire during the guard's ratchet.

**What I did:**

- `src/actuators/cmd.rs`: promoted cpu.rs's private `ryzenadj --info`
  table-building test helpers into `cmd::test_support` as
  `ryzenadj_info_table` (the table-text builder), `output_with_stdout` (an
  `Output` wrapper), and `queue_ryzenadj_readback` (queues one full write +
  read-back `FakeRunner` cycle reporting an arbitrary table). The existing
  `synthesize_ryzenadj_info` (the auto-agreeing default) now builds its
  table via the same shared function instead of its own private copy.
  `cpu.rs`'s own `info_output`/`info_table_text` test helpers now delegate
  to these shared versions (kept as thin local aliases so cpu.rs's many
  existing call sites needed no rename) — this is what makes it possible
  for `controller.rs`'s tests to script a genuine `WriteVerdict::Mismatch`
  through the real `CpuActuator::set_sustained_mw` write path, which is
  exactly what the review's "could not find or build a scripting helper"
  gap was about.

- `src/control/controller.rs`, two new controller-level tests:
  - `cpu_mismatch_freezes_flags_reasserts_and_releases_to_stock_then_a_later_verified_recovers`:
    drives `auto_controller` through TempLoop with three scripted confirmed
    `Mismatch`es (each needing BOTH the initial write attempt and its
    re-read to disagree, since `run_budget_and_allocate` discards the
    first verdict's value whenever the first read is itself a `Mismatch`
    and feeds `VerdictState::observe` only the second call's result). This
    exercises the real write -> re-read -> observe -> apply_verdict_outcome
    chain, not `VerdictState`'s state machine directly. Asserts, tick by
    tick: the first confirmed mismatch sets `LimitNotSticking` but cannot
    freeze its OWN tick's budget (the freeze decision necessarily precedes
    the write that produces the verdict — structurally one tick behind, as
    the existing code comments already documented); the second confirmed
    mismatch's tick IS frozen (`actuator_mismatch`) by the first's
    now-in-progress episode; the third confirmed mismatch releases to
    stock (`cpu_limit_w` becomes `None`, `restore_stock` runs against a
    real profile-file fixture); the released actuator keeps reasserting on
    its own every due tick, and the NEXT write — deliberately left
    unscripted — hits `FakeRunner`'s ordinary auto-agreeing default and
    recovers (`cpu_limit_w` becomes `Some` again), proving recovery through
    the *ordinary* write path rather than a hand-picked scripted agreement;
    a further tick confirms the freeze no longer reflects the resolved
    episode.
  - `gpu_hot_episode_ratchets_the_cap_without_spuriously_triggering_demand_limited`:
    replays a GPU-HOT episode at controller level — `guard_state.gpu_hot`
    -> `gpu_share_override` -> `gpu_max_w` fed into
    `split_budget`/`last_gpu_cap_w` — which the idle-tick anti-windup test
    never touches at all. Warms up a real (non-hot) cap over 100 RpmLoop
    ticks with both axes tracking their own last RAW (pre-slew,
    post-guard-override) split cap closely (a "fully loaded, never idle"
    GPU and CPU), confirms the warm-up actually built a cap comfortably
    above the LUT floor, then flips the dGPU guard hot and drives 40 more
    ticks the same way. Asserts the ratchet actually walks `gpu_w` down to
    the LUT floor over the episode (confirming the guard's wiring is live,
    not just "never fires"), and that the demand-limited predicate's
    `freeze == "demand_limited"` never appears while both axes keep
    consuming whatever cap they are handed — the scenario a bug like
    judging demand against a stale pre-override cap would show up in.

**Debugging notes (both tests needed real iteration against the running
implementation, not just hand-derivation):**

- The CPU-mismatch test's write cadence starts at `t=0`, not after
  TempLoop's entry hysteresis finishes: `due` (the 5 s allocate/write
  cadence) fires on the first sample regardless of mode, and `mode` is
  already `RpmLoop` (not `Released`) from `t=0` since `fan_valid` is true
  from the first sample — `need_write`'s `mode != Released` gate is
  satisfied immediately. My first draft assumed the first write landed at
  `t=5`; the actual schedule is `t=0, 5, 10, 15, 20`.
- `temploop_sample`'s own default `cpu_temp_valid: false` is fine for its
  existing ~6-sample callers but trips `ThermalWatchdog`'s
  `SENSOR_LOST_STREAK` (10 consecutive invalid-Tctl samples) partway
  through this test's longer run, forcibly releasing everything via
  `emergency_release` before the scripted mismatch chain finished — fixed
  by setting `cpu_temp_valid: true` on every sample.
- The GPU-HOT test's first draft used `gpu_temp_c: 95.0` throughout the hot
  phase, which also tripped `ThermalWatchdog`'s SEPARATE `GPU_TRIP_C`
  (87°C, `TRIP_STREAK=3`) — `GPU_HOT_C_DEFAULT` (the guard's own enter
  threshold, 90°C) sits ABOVE the watchdog's independent emergency
  threshold, so any temperature that latches the guard hot also trips the
  unrelated thermal emergency within 3 samples and releases everything.
  Fixed by latching the guard hot with one sample at exactly 90°C, then
  holding at 86°C for the rest of the episode — inside the guard's own
  hysteresis band (exit is enter−5=85, so 86 keeps it latched) but below
  the watchdog's 87°C trip, so the watchdog's hot-streak keeps resetting to
  0 every tick instead of accumulating.
- The GPU-HOT test's first draft tracked each axis's draw against the
  PREVIOUS tick's slew-clamped committed watts (`Effect::AutoAllocated`'s
  `cpu_w`/`gpu_w`, or `ctl.status().cpu_limit_w`) rather than the RAW
  pre-slew cap the demand-limited predicate actually judges against
  (`AutoState::last_cpu_cap_w`/`last_gpu_cap_w`). Because the GPU guard's
  down-ratchet (`DOWN_RATE_W`, 8 W/allocator-tick) frees surplus for the
  CPU axis faster than the allocator's own CPU up-slew (`UP_RATE_W`, 2
  W/tick) can commit it, the RAW CPU cap runs ahead of the slew-limited
  commit during the descent, and a draw tracking the slow commit
  spuriously showed up as CPU-axis demand-limited — an artifact of the
  test's own modeling gap, not the controller's. Confirmed via `eprintln!`
  tracing (removed before the final version) showing `freeze="demand_limited"`
  firing with the GPU axis's own cap/draw gap well inside its margin,
  pointing at the CPU axis instead. Fixed by reading both axes' RAW caps
  directly from `AutoState`'s private fields each tick (accessible from
  the `tests` submodule via ordinary Rust child-module visibility) and
  tracking draw against THAT, matching exactly what the predicate itself
  reads.

**Verification run:**

```
cargo test --bin fw-fan-quiet        # 564 passed, 0 failed, 2 ignored (pre-existing, hardware-gated, unrelated)
cargo clippy --all-targets            # no new warnings (same pre-existing dead-code set as the original report)
cargo fmt --check                     # clean for all three touched files (cmd.rs, cpu.rs, controller.rs);
                                       # fopdt.rs/plant.rs's pre-existing unrelated drift (noted in the
                                       # original report) is untouched -- a stray `cargo fmt --` run without
                                       # scoping briefly reformatted them; reverted with `git checkout --`
                                       # before committing so this round's diff stays scoped to the finding.
```

**Files changed this round:**

- `src/actuators/cmd.rs` — shared `ryzenadj --info` table-scripting helpers
  in `test_support` (`ryzenadj_info_table`, `output_with_stdout`,
  `queue_ryzenadj_readback`); `synthesize_ryzenadj_info` now builds its
  table via the same function.
- `src/actuators/cpu.rs` — `info_output`/`info_table_text` test helpers now
  delegate to the shared `cmd::test_support` versions instead of keeping
  their own private copies.
- `src/control/controller.rs` — two new controller-level tests (above).

**Status:** FIXED
**Head:** `8d7b987dcb9879949b8c7dd7386ada087f55da61`

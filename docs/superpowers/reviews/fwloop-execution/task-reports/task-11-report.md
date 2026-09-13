# Task 11 report: Spike: settle the anti-windup rule (fw-fanctrl-loop-9it)

**Status: IMPLEMENTED.** Re-dispatch after two prior attempts (run 1: BLOCKED on the ryzen_smu
precondition, then filed a blocker bead; run 2: closed without doing the work due to a
brief-stage `alreadyMerged` false positive — both documented on the bead's comment thread, which
I read first per instructions). This run completed the measurement, built the harness, ran the
sweep, and rewrote §2.4.

## Summary

The CPU-axis `ryzenadj` precondition that blocked run 1 was **not** a genuine blocker: the fix
(per the bead's clarification comment) is to unload `ryzen_smu` and **leave it unloaded** for the
whole measurement — run 1 unloaded and immediately reloaded it via `modprobe -r ryzen_smu &&
modprobe ryzen_smu`, which restores the broken precondition before `ryzenadj` ever runs. I
unloaded it, confirmed `ryzenadj --info` worked (root), ran the full CPU measurement, then
restored the module afterward. Both axes' `DEMAND_MARGIN_W` are now real, on-machine
measurements (see §Facts and below), not assumed values.

With real margins in hand, I built the throwaway harness (`src/control/spike_antiwindup.rs`),
swept four candidate rules against all seven named scenarios using the **real** `Budget`, a
first-order plant driven by actual drawn watts, and the real `allocator::split_budget`, then
rewrote design doc §2.4 with the decided rule, its constants, and the sweep table.

## The decision (also in §2.4)

**Per-axis `ConditionalHysteresis`.** Per axis `i`, per tick:

```
demand_limited(i) := cap_i > floor_i + ε   AND   (cap_i − draw_i) > DEMAND_MARGIN_W(i)
```

- `cap_i` is the **post-guard-override** cap (what `split_budget` actually produced this tick,
  after `gpu_share_override`'s ratchet has been applied to `gpu_max_w`) — there is only one cap
  an axis is ever actually offered, so that is the only one worth comparing against.
- `floor_i` is the axis's own configured floor. The `cap_i > floor_i + ε` guard is the load-bearing
  part of "judge each axis separately": it is what excludes an axis parked exactly at its floor
  (a structurally-undrawn GPU) from ever registering a false "wasting headroom" signal — a plain
  per-axis comparison *without* this guard would still fail scenario 5 the same way a combined-sum
  test does. This is the actual finding of the spike, not just "loop over axes instead of summing".
- Halt = `any(demand_limited(i)) AND error_sign > 0` — one axis is enough to gate the shared
  scalar `u`, only in the deepening direction. `Budget::step`'s existing `Freeze::DemandLimited`
  handling (unchanged, out of this task's `filesTouched`) already implements that gate correctly.
- **Hysteresis: 2 ticks (10 s)**, symmetric entering/leaving. Short relative to `Ti = 35 s`, and
  the sweep's scenario 7 shows it cuts hysteresis-state chatter from 81 raw transitions to 29.
- **Leaving the hold needs no bespoke resync rule.** `Budget::step`'s existing generic
  "leaving any freeze" resync already fires correctly on every `DemandLimited -> not` transition.
- **`GPU HOT` needs no `Budget`-level freeze.** Feeding the guard's already-ratcheting
  `gpu_max_w` into `split_budget` is enough — its existing surplus reassignment automatically
  hands the GPU's shrinking share to CPU. Confirmed by scenario 6 (all four candidates identical
  — no candidate adds a spurious extra hold on top of the guard).

**`DEMAND_MARGIN_W` (measured 2026-09-09, on this machine — full method in §Facts):**
- **CPU: 2.0 W.** `ryzenadj --info`'s `PPT VALUE SLOW` vs `PPT LIMIT SLOW`, RAPL
  `energy_uj`-cross-validated: noise floor ≤ 0.08 W across four cap-bound load compositions
  (24-thread saturating stress-ng at 20 W/35 W caps, lighter loads at 35 W — this machine's own
  desktop background draw turned out to already exceed 35 W, so those "light" tests stayed
  cap-bound rather than demand-limited). A genuine demand-limited gap, measured directly (54 W
  cap vs 36.8 W actual desktop draw), was 17.2 W. 2.0 W: ~25x the noise floor, ~8x below the
  smallest genuine gap.
- **GPU: 3.0 W.** NVML `power.draw` at a 1500 MHz clock lock: a `glxgears` fill/geometry-bound
  proxy load gave a 0.11 W spread (stated limitation: no heavier compute-bound generator was
  available, so this likely understates the real noise floor). A genuine gap, measured directly
  (idle 7.27 W vs the same proxy-loaded 13.4 W at the same lock), was 6.1 W. 3.0 W: ~27x the
  noise floor, about half the smallest genuine gap — proportionally more conservative than the
  CPU margin, since its own noise-floor number is the weaker measurement.

## The sweep (tabulated fully in §2.4; `cargo test control::spike_antiwindup -- --nocapture`)

| Scenario | NoHalt | Conditional | **ConditionalHysteresis (chosen)** | CombinedSum |
|---|---|---|---|---|
| 1 idle wind-up then load onset | overshoots target by 14.0 °C-equiv (windup) | holds | **holds** | holds |
| 2 lull mid-session | self-latches | recovers | **recovers** | recovers |
| 3 warm-start, lighter load, EC above T* | holds | holds | **holds** | holds |
| 4 mid-session target drop | holds | holds | **holds** | holds |
| 5 structurally undrawn axis | holds | holds | **holds** | holds, measurably worse (see note) |
| 6 GPU HOT, CPU at its own cap | holds (identical across all four) | holds | **holds** | holds |
| 7 oscillation around the margin | holds (no halt ever triggers) | holds, 81 transitions | **holds, 29 transitions** | holds (no halt ever triggers) |

No candidate ever pulled `u` toward the draw (`cap_tracking` false in every one of the 28 runs).

**Scenario 5 honesty note** (also in §2.4): on this machine's specific measured margins, the
closed-loop degradation of `CombinedSum` vs per-axis in scenario 5 is real but small (0.063 vs
0.060 °C-equiv settled error) — smaller than the design doc's "CPU pinned at its floor" framing
would suggest, because the GPU's structural floor share (5 W in the harness) happens to sit close
to `margin_sum` (2.0+3.0=5.0 W) for these specific measured constants. Rather than tune the
harness's numbers to manufacture a bigger gap, I added two **predicate-level** tests
(`per_axis_predicate_excludes_an_axis_pinned_at_its_floor`,
`combined_sum_predicate_mis_flags_a_perfectly_tracked_cpu_next_to_a_pinned_gpu_floor`) that
demonstrate the structural failure directly and unambiguously, independent of this specific
machine's margin/floor coincidence. Both are documented in §2.4.

## Files changed

- `src/control/spike_antiwindup.rs` (new, `#[cfg(test)]`): the harness — `Xorshift32` RNG,
  `Plant` (FOPDT), `Rule` enum (4 candidates), `demand_limited_axis`, `RuleState` (hysteresis
  dwell + fair toggle counting across all four rules), 7 scenarios as data (`Scenario` struct +
  `scenario_table()`), `run_scenario`, `sweep`, `render_sweep_table`, and 9 tests.
- `src/control/mod.rs`: one line, `#[cfg(test)] pub mod spike_antiwindup;`.
- `docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md`: §2.4 rewritten (the "open for
  the spike to decide" paragraph replaced by the decided rule, its constants, and the sweep
  table + notes; the four fixed invariants and the "why prose failed" history are unchanged);
  §Facts gets the `DEMAND_MARGIN_W` measurement (numbers, method, date).

## Test results

`cargo test --bin bazerame-fans`: **543 passed, 0 failed, 2 ignored** (full suite, confirms
nothing else broke).

`cargo test --bin bazerame-fans control::spike_antiwindup`: **9 passed, 0 failed** —
`no_candidate_ever_tracks_the_draw`, `no_halt_baseline_overshoots_the_windup_scenario`,
`per_axis_rules_overshoot_much_less_than_no_halt_on_the_windup_scenario`,
`per_axis_predicate_excludes_an_axis_pinned_at_its_floor`,
`combined_sum_predicate_mis_flags_a_perfectly_tracked_cpu_next_to_a_pinned_gpu_floor`,
`hysteresis_reduces_chatter_on_the_oscillation_scenario`, `winner_holds_every_scenario`,
`print_sweep_table`, `gpu_hot_reassigns_its_discarded_watts_to_cpu_via_split_budget`.

`cargo clippy --tests -- -D warnings`: clean. (Plain `cargo clippy --all-targets -- -D warnings`
fails with 46 pre-existing dead-code errors unrelated to this change — confirmed via `git stash`
against the base commit `247c2d1`, same failures present before I touched anything. `--tests`
matches the scope `cargo test` itself builds, which is the correct gate for a `#[cfg(test)]`-only
module.)

## TDD evidence

This is a measurement spike, not a feature with a pre-existing failing-test contract — the
"RED" state was the *absence* of a decided rule (§2.4's open list) and the harness itself was
built and iterated against real numeric output (the sweep table), not a single red/green pair.
Concretely: I compiled and ran the harness repeatedly while designing each scenario, watching the
sweep table's actual numbers (not assumed ones) to find scenario parameters that exercise the
real failure modes — e.g. scenario 1's initial parameters didn't show the windup at all until I
extended the run length to let the τ=35s/θ=20s plant actually settle (the closed-loop time
constant here is λ≈150s per §2.4's own IMC derivation, so a short scenario just doesn't have time
to show anything). The final `no_halt_baseline_overshoots_the_windup_scenario` and
`winner_holds_every_scenario` tests are the closest thing to a red/green pair: run against the
*first* (too-short, too-tight-tolerance) scenario set, all four candidates failed for reasons
unrelated to the rule under test (plant settling time, not anti-windup); after fixing the
scenarios to be physically self-consistent (see "self-review" below), `no_halt` fails exactly the
windup scenario as expected and `ConditionalHysteresis` passes all seven.

## Self-review

Several real bugs found and fixed while building this, worth recording because they'd matter to
anyone re-deriving the numbers:

1. **First scenario set was too short.** `Ti=35s` PI dynamics need ~100+ ticks (500+ s) to show
   anything meaningful; my first pass used 80-110 tick scenarios and every candidate "passed"
   trivially because nothing had converged yet in either direction. Fixed by extending to
   200-400 ticks and re-deriving tolerances from the IMC λ the design's own gains target.
2. **Unreachable targets, not windup, were failing scenarios 3/4/7 initially.** I set targets
   requiring more power than the scenario's own "true demand" could ever supply (e.g. target
   needing 56 W with only 42 W of demand specified) — every candidate "failed" identically because
   the plant physically couldn't get there, which is a scenario-design bug, not a finding about
   anti-windup. Fixed by deriving each target from `K * demand_total` so it's actually reachable,
   and (for scenario 7) decoupling the demand SCORE fed to `split_budget` from the DRAW target,
   since I needed the score to saturate the cap at the hardware ceiling while the draw stayed a
   controlled distance below it.
3. **A GPU floor that dropped to 0 when "unpowered" silently fixed the exact bug scenario 5 exists
   to expose** — `split_budget`'s floor argument is a policy constant (the LUT watts at the
   configured clock floor) that doesn't know about power state; only the DRAW is zero when
   unpowered. I had accidentally modeled the floor itself as conditional on power state, which
   made the "structurally undrawn" gap disappear from my own model.
4. **Peak-overshoot grading was scenario-blind at first**, flagging scenarios 3/4/6 as "failures"
   because `y` legitimately starts (or is driven) above the CURRENT target by the scenario's own
   setup (a warm-start above T*, a target that just dropped) — not a windup artifact. Added a
   `check_overshoot` flag so only scenario 1 (the one actually testing windup-then-overshoot)
   grades on it.
5. **The `toggles` chatter metric only counted transitions for the two candidates that route
   through the hysteresis-debounce helper**, silently reading 0 for `Conditional` regardless of
   actual chatter. Fixed to count transitions on the final per-tick verdict uniformly across all
   four rules, which is what actually made scenario 7 a meaningful comparison (81 vs 29, not 0
   vs 29 which would have overstated hysteresis's benefit).

**Assertion discipline check** (per the runtime instructions): every assertion in the 9 new tests
names a concrete value the harness could produce that would fail it. The one I scrutinized hardest
is `per_axis_predicate_excludes_an_axis_pinned_at_its_floor`'s first assertion
(`!demand_limited_axis(0.0, floor, floor, DEMAND_MARGIN_W_GPU)`, `floor = 50.0`) — this is NOT
decoration: `demand_limited_axis`'s `cap_w > floor_w + 1e-9` guard is exactly what makes it false
despite the huge nominal gap (`cap - draw = 50`), and a version of the function without that guard
(which is precisely the bug this task is about) would make it fail. I ran it mentally against both
implementations to confirm.

## Concerns for the coordinator

None outstanding for this task. `fw-fanctrl-loop-j6s` (Controller loop integration) can now read
a genuinely rewritten, implementable §2.4 rather than an open list — per the epic-specific
constraint, it should still read §2.4 itself rather than trust this report's summary.

The blocker bead `fw-fanctrl-loop-2ll` filed by run 1 (before this run's clarification arrived)
should probably be closed by whoever triages it next — the CPU measurement it reported as
unavailable is done and recorded in §Facts now. I did not close it myself since bead lifecycle
management outside this task's own bead is not something I was asked to do, and the coordinator
or a triage pass is the usual place for that per this project's workflow.

---

## Fix round 1/5 (review finding: stale budget.rs ownership doc comments)

**Status: FIXED.**

### Finding

`src/control/budget.rs`'s module docs (lines 24-25 as reviewed) and
`Budget::set_demand_state`'s doc comment (lines 248-250 as reviewed) still said
`fw-fanctrl-loop-9it` "owns the real rule and rewrites this section and its callers" /
"rewrites this function (and §2.4) accordingly." This task's diff never touched
`budget.rs` and never called `set_demand_state` — the decided predicate
(`demand_limited_axis`, needing `draw`, `cap`, `floor` and a per-axis `DEMAND_MARGIN_W`)
lives only in the throwaway `#[cfg(test)]` harness and is structurally incompatible with
`set_demand_state`'s production signature (`&[(f64, f64)]` draw/cap pairs + `error_sign`,
no floor, no margin — its body is still the naive `draw >= cap` pin check the spike's
floor-guard fix, scenario 5, was built to replace). The spec's own plan table treats
`Budget::set_demand_state` as the literal seam `fwloop.12`/`fw-fanctrl-loop-j6s` will call
to wire in the decided rule, but that seam cannot carry it as typed, and the report's
"Concerns for the coordinator: None outstanding" did not flag the gap.

### Root cause

`budget.rs` is genuinely outside this task's `filesTouched` (per `task-11-brief.md`:
`src/control/spike_antiwindup.rs`, `src/control/mod.rs`, and the design doc only), so the
prior run correctly left the file untouched during the main pass. But its two doc comments
are now stale artifacts of an earlier plan draft (before the spike/wiring split existed)
and were never updated to reflect that split, and the report should have surfaced the
resulting seam-signature gap explicitly rather than declaring nothing outstanding.

### Fix

Edited both doc comments in `src/control/budget.rs`:

1. **Module docs (`# Anti-windup` section).** Replaced "The concrete predicate is a
   placeholder; `fw-fanctrl-loop-9it` ... owns the real rule and rewrites this section and
   its callers" with a note that `fw-fanctrl-loop-9it` has now decided the rule and
   recorded it in §2.4 (stating the predicate and hysteresis inline), that
   `set_demand_state` below still implements only the older placeholder because its
   signature has nowhere to carry a floor or a per-axis `DEMAND_MARGIN_W`, and that
   `fw-fanctrl-loop-j6s` (Controller loop integration) owns widening the seam and its
   callers to the decided rule.
2. **`Budget::set_demand_state` doc comment.** Replaced "Placeholder — owner:
   `fw-fanctrl-loop-9it`" with "Placeholder — owner: `fw-fanctrl-loop-j6s`", restated the
   decided predicate and its constants (per-axis `cap_i > floor_i + ε AND (cap_i − draw_i)
   > DEMAND_MARGIN_W(i)`, 2-tick symmetric hysteresis, post-guard-override cap) as what
   `fw-fanctrl-loop-9it` already decided and recorded, and spelled out explicitly why the
   current `&[(f64, f64)]` signature cannot express it (no floor, no per-axis margin) and
   what `fw-fanctrl-loop-j6s` — named as the seam's documented caller in the plan's
   `fwloop.12` row — must widen it to carry (each axis's `floor_i`, `DEMAND_MARGIN_W(i)`,
   and hysteresis state) before it can wire in §2.4's rule.

No production logic changed — `set_demand_state`'s body and signature are unchanged, since
widening them is `fw-fanctrl-loop-j6s`'s task, not this one; this fix only corrects the
doc comments' stale ownership claim and makes the signature mismatch explicit so the
coordinator (or `j6s`'s implementer) doesn't discover it late.

### Verification

- `cargo check --bin bazerame-fans`: clean (only the same pre-existing 46 dead-code
  warnings noted in the original report, confirmed unrelated via `git stash` in that run).
- `git diff -- src/control/budget.rs` reviewed: doc-comment-only change, no code touched.

### Files changed this round

- `src/control/budget.rs` (doc comments only, both flagged locations).

### Commit

`dce58700e366e0f219f1b660f55dfed03ae69213` — "fix(control): correct stale ownership doc
comments in budget.rs (task 11 review round 1)".

`git rev-parse HEAD` after this fix: `dce58700e366e0f219f1b660f55dfed03ae69213`.

# Final whole-epic review — run wf_8e35e7de-079 (2026-09-09)

stopReason: ready-drained  |  completed: 15  |  escalated: ['fw-fanctrl-loop-zct', 'fw-fanctrl-loop-dsh']
parked: []  |  pendingRetry: []  |  stalled: False

## Metrics lines (NOTE: all counters read zero — coordinator defect, see friction log item 9)

    Metrics: merges 0 · merge-failed 0 · rebase-conflicts 0 · seam-reviews 0 (fixed 0) · gate-fails 0
    Metrics: fix-loop round 1: 0 addressed / 0 entered · round 2: 0 addressed / 0 entered · round 3: 0 addressed / 0 entered · round 4: 0 addressed / 0 entered · round 5: 0 addressed / 0 entered
    Metrics: fix-loop breaker-tripped: 0
    Metrics: ledger-check M≠completed: 0 vs 15 · append-failed 0 · append-retried 0

## Sweep

    9f112ed — 0 passed, 0 failed, 83 errors, 0 skipped; failing: src/actuators/mod.rs:33:5, src/actuators/gpu.rs:32:7, src/actuators/gpu.rs:36:7, src/actuators/gpu.rs:40:7, src/actuators/gpu.rs:50:12, src/actuators/gpu.rs:57:12, src/calib/fopdt.rs:46:11, src/calib/fopdt.rs:48:11, src/calib/fopdt.rs:52:7, src/calib/fopdt.rs:55:7, src/calib/fopdt.rs:57:7, src/calib/fopdt.rs:61:7, src/calib/fopdt.rs:62:7, src/calib/fopdt.rs:69:12, src/calib/fopdt.rs:90:8, src/calib/fopdt.rs:118:4, src/calib/fopdt.rs:165:4, src/calib/fopdt.rs:223:8, src/calib/fopdt.rs:236:4, src/control/allocator.rs:307:12; command: cargo clippy --all-targets -- -D warnings — not in this measurement (escalated or pending retry, never merged): fw-fanctrl-loop-zct, fw-fanctrl-loop-dsh

## Review

## VERDICT: **DO NOT LAND.** This is a mid-demolition branch, not a finishable one.

Branch: `epic-fw-fanctrl-loop-6ma-integration` @ `9f112ed` (worktree `/var/home/alepar/AleCode/fw-fan-quiet/.claude/worktrees/fw-fanctrl-loop/.worktrees/epic-fw-fanctrl-loop-6ma-integration`)

**Measurements I re-ran at the tip (the sweep is valid, not stale):**
- `cargo test` — 599 passed, 0 failed, 2 ignored. **Green.**
- `cargo clippy --all-targets -- -D warnings` — 83 errors, **100% `dead_code`** (I filtered: zero non-dead_code errors). Matches the reported sweep exactly.

The green test result is the trap. It is green *because* task 12 deleted the tests that would fail.

---

## 0. Headline: the epic is 14/24 landed and the branch ships a dead Auto mode

`bd list` — 9 real tasks still open (`zct`, `dsh`, `0nv`, `438`, `7ij`, `cm7`, `eyi`, `j6s`, `nsc`) plus 2 blocker beads (`6ma.1`, `bwt`). The one that wires the epic in — `fw-fanctrl-loop-j6s` "Controller loop integration" (fwloop.12) — has not run.

Consequence, verified in source:
- `master:src/control/controller.rs:1070` had a live contour: `let contour = |pc| model.gpu_watts_on_contour(target_rpm, bias, gain, pc);`
- HEAD `src/control/controller.rs:1097` has: `let contour: &dyn Fn(f64) -> Option<f64> = &|_pc: f64| None;`
- `src/control/allocator.rs:380` — `else { return prev; // degenerate contour everywhere → freeze }`

So on this tip Auto mode raises to the CPU floor and freezes there forever. `grep gpu_watts_on_contour src/` returns only test callers. Task 24s removed the old (working) adaptation tier; task 6 (`zct`) which supplies the replacement scalar budget split **never merged**. Landing this is a functional regression from `master` with a green test suite as cover.

The whole new stack — `Budget`, `Arbiter`, `Fopdt`, `DutyRpmTable`, `GpuLockVerifier`, `Curve`, `EcAverage`, `WarmStart`, `thermal_model` — has **no production caller**. That is the entire 83-error clippy sweep.

---

## 1. Recurring clusters — the detector under-fired; here are the real ones

The ledger has **zero `Recurring minor:` and zero `Recurring blocker:` lines**, and the Metrics block is visibly broken:

```
Metrics: merges 0 · merge-failed 0 · rebase-conflicts 0 · seam-reviews 0 (fixed 0) · gate-fails 0
Metrics: fix-loop round 1: 0 addressed / 0 entered ... round 5: 0 addressed / 0 entered
```

The ledger body records 15 merges, 2 rebase conflicts, 3 `gate fail → blocker`, 3 `seam-review cleared/fixed`, and 2 fix rounds that each addressed 1 finding. **All counters read 0.** The recurring-detector reads those counters, which is why it fired nothing. Treat the absence of `Recurring` lines as instrument failure, not as absence of clusters. Three clusters are present by inspection:

**Cluster A — "dead_code because the wiring task never ran" (3 tasks: 834, jpg, 24s; 83 sweep errors).** One class, one cause: fwloop.12/`j6s` is unrun. **Do not fix instance-by-instance.** Any `#[cfg(test)]` or `#[allow(dead_code)]` patch would be a lie that has to be reverted by `j6s` and would re-open the mod.rs hot-file conflict the plan warns about. Disposition: **not addressable on this branch — it is the branch's incompleteness, reported as 83 nits.**

**Cluster B — stale comments referencing removed/not-yet-wired symbols (4 tasks: fo1, 51b, mjv, 24s).** `src/control/trim.rs:117` and `src/control/trust.rs:7` still cite `StatusFlag::ModelDistrust` (deleted); `src/test_support/mod.rs` still says the `plant` module is an "Empty stub"; `controller.rs:288` `flag_severity()` classifies `NvmeHot` as `Info` while design §3.5 and the new `view.rs` render it as *warning*. This class has an owner already: **`fw-fanctrl-loop-eyi` (Deletion sweep), still open.** Disposition: **assign the class wholesale to `eyi`; do not fix individually.** The `flag_severity` one is the sharp edge — it is `#[allow(dead_code)]` today, so it is a latent trap for whoever wires it, not a live bug.

**Cluster C — tautological assertions (2 tasks: blm, iym).** `src/test_support/fixtures.rs` asserts `p.is_absolute()` on a path always joined onto `CARGO_MANIFEST_DIR`; `src/control/mode.rs:646` asserts `d.reasons.contains("unreconciled") || d.mode != LoopMode::TempLoop` where the RHS is unfailable (entry needs 3 ticks, `entry_streak` starts 0). Below the ≥3 threshold but the same smell: reviewers accepting assertions that cannot fail. Disposition: **no-action on this branch; one line in the reviewer brief for the remaining 9 tasks.**

---

## 2. Root cause of both escalations: the hot-file lock never bound

```
Detector: round 1 — ... hot-file deferrals: none (expected serialisation point src/control/controller.rs did not bind)
```

The coordinator says it plainly. `controller.rs` was supposed to serialize tasks 6/12/13/15/19; it didn't, so `zct`, `24s`, `dsh` and `mjv` all raced on it. Every escalation in this run is that one defect: `zct` blocked (its deletions kill tests `24s` owns), `dsh` blocked (`bwt`: PersistedState change breaks ~50 tests `24s` owns), `mjv` a ~1700-line rebase conflict on the same file. **This is a pipeline defect, not three task defects.** Re-dispatching any of them without a `blocks` edge re-blocks deterministically — both triage agents said so explicitly.

---

## 3. MUST-FIX before this branch can land

### B1 — `Curve::t_star()` returns ±∞ at the floor/ceiling duty and permanently NaN-poisons the integrator. **CONFIRMED with a repro I ran.**

This is the deferred minor from task 1 (`9dv`): *"tread()'s NEG_INFINITY/INFINITY bounds … a literal-but-unverified reading of §2.1, needing the design author's sign-off before a downstream setpoint/PI consumer relies on it."* Task 16 (`iym`, `mode.rs`) then merged **as that consumer, with no guard.** No per-task review could see both ends. `grep is_finite` over `mode.rs`, `curve.rs`, `budget.rs` → nothing.

`src/fanctrl/curve.rs:171-188` returns `(-inf, hi)` / `(lo, +inf)` at the floor/ceiling duty; `curve.rs:192` `t_star = (lo+hi)/2.0` → ±∞. `src/control/mode.rs:349` takes it unguarded into `Decision.t_star`. I appended a throwaway probe to `budget.rs`, ran it, and reverted (tree confirmed clean via `git status --porcelain`):

```
PROBE floor tread=Some((-inf, 56.666666666666664)) t_star=Some(-inf)
PROBE ceil  tread=Some((80.0, inf))                t_star=Some(inf)
PROBE tick 0: e=inf u=100  at_upper_for=5s
PROBE tick 1: e=inf u=NaN  at_upper_for=0ns
PROBE tick 2: e=inf u=NaN  at_upper_for=0ns
PROBE tick 3: e=inf u=NaN  at_upper_for=0ns
```

`src/control/budget.rs:295` — `raw_du = kc * (e_k - self.e_prev) + …` gives `inf - inf = NaN` on tick 1. `NaN.clamp()` propagates, so `u` is NaN forever with no recovery path (`e_prev` stays `inf`, `v` stays NaN). Worse: `at_upper` is `v_new >= hi`, false for NaN, so `upper_bound_dwell` **resets to 0** — the §2.7 `high` TARGET UNREACHABLE rule never fires. The loop dies silently, unflagged, and hands NaN watts toward `ryzenadj`.

Both ends are reachable: ceiling duty is what `duty_for_rpm` returns for a max-fan target; floor duty (`t_star = -inf`) makes `-inf >= max_unc + 5` false, so the quietest duty is latched permanently infeasible. `mode.rs`'s own test at line 824 passes *by accident* off exactly this.

Fix belongs at the curve/arbiter seam (finite clamp to the curve's endpoint, or `t_star -> None` on an infinite tread plus an explicit reason), **not** in `j6s`. Design §2.1 needs the sign-off task 1 asked for and never got.

### B2 — `fw-fanctrl-loop-9it` (fwloop.24, the anti-windup spike) is **closed with none of its deliverables landed.** Phantom completion.

Ledger: `Task 11 (fw-fanctrl-loop-9it): complete (already merged into epic-… before this re-entry — bead closed, no new review)`. `bd show` confirms `✓ … CLOSED`. Nothing merged:
- `src/control/spike_antiwindup.rs` does not exist and **appears in no commit on any branch** (`git log --all -- <path>` is empty).
- §2.4 still carries the verbatim open list at `docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md:303-308`: *"Open for the spike to decide and record: the predicate itself; … `DEMAND_MARGIN_W` per axis …"* — the exact text the acceptance criterion says must be replaced.
- `DEMAND_MARGIN_W` appears nowhere except that open list and a `budget.rs:262` comment saying it is deliberately not tuned there.
- §Facts has no commanded-vs-drawn measurement.
- No candidate × scenario table.

fwloop.12 (`j6s`) is `blocked-by fwloop.24` with the note *"the wiring cannot be written before the rule exists."* **`j6s` is currently unblocked in beads and would be dispatched against a rule that does not exist.** Re-open `9it`; the closed-bead-means-done inference is the pipeline bug that let it through.

### B3 — Clippy sweep is red (83 errors) and cannot be made green on this branch.

Stated for completeness: this is Cluster A, and the only correct fix is landing `j6s`. Flagging it so nobody "fixes the sweep" with allow-attributes.

---

## 4. Deferred minors — full triage (27 across 12 tasks)

**Must fix before land:** the one above (B1, from `9dv`). Everything else, by disposition:

**Fold into an already-open task (do not fix here):**
- `fo1`, `51b`, `mjv`, `24s` stale-comment set → **`eyi`** (Cluster B).
- `jpg`: `GpuLockVerifier::verify_lock` unwired → **`j6s`**, which already owns it.
- `4aj`: `fit_fopdt` takes an extra `min_response: f64` beyond the design pseudocode's 2-arg signature; and `search_tau_theta`'s grid bounds (`tau_hi=3*span`, `theta_hi=0.5*span`) are validated against one synthetic scenario → both are **`0nv`**'s (fwloop.18) problem when it meets real fixture data. Genuine open risk; note in the brief.
- `51b`: `ThermalPlant::tick()`'s `EcReading::read(&self.dir).expect(...)` panics if all EC channels are scripted ≤ 0.0 with `gpu_ec` unset → **`cm7`** (fwloop.17), which is the task that will script those runs.

**Fix cheaply whenever `budget.rs` / `mode.rs` are next touched (all pre-`j6s`, none land-blocking):**
- `834`: `at_lower_bound_for`/`at_upper_bound_for` (`budget.rs:327-338`) use independent `<=`/`>=`, so `lo==hi` (the pre-`set_bounds` state) double-counts. One-line guard.
- `834`: `step()` (`budget.rs:292-300`) only calls `resync_error` implicitly on a *kind* switch or on leaving freeze entirely. A `Freeze::ActuatorMismatch → Freeze::DemandLimited` transition with no unfrozen tick between skips it, so `DemandLimited`'s first tick kicks off a stale `e_prev`. **Verified this is genuinely unspecified** — the spec's fwloop.4 row (line 733) requires only *"an error-kind switch and leaving any freeze"*. Its natural owner is the spike (B2), which never ran.
- `iym`: `mode.rs:537` `flags.dedup()` removes only *consecutive* duplicates; correct today only because the four `TargetUnreachable` push sites happen to be mutually exclusive. Implicit invariant, one edit from breaking.

**No action — correctly deferred, and I agree:**
- `blm` + `iym`(a): tautological assertions (Cluster C).
- `9dv`(a): `duty.saturating_add(1)..` self-includes at `duty==255`; unreachable outside the 0–100 domain.
- `58u` ×3: `Option<Instant>` with a structurally unreachable `None`; `FakeFanctrl` not asserting outcome/command agreement; `connect_with_timeout`'s thread leak on a hung AF_UNIX `connect`. All correctly characterised as ceremony, not defects.
- `jpg`(c,d): `gpu_sm_mhz` field naming; `ryzenadj` stderr reaching only `tracing::warn!`. Inherent to the richer verdict type.
- `sov` ×3: `find_chip_dir` duplicating `Hwmon::discover`; an inaccurate rationale sentence in a report; mutex-poisoning cascade that mirrors existing convention.
- `24s` ×2: an extra inline comment at the `AllocInput{contour}` site; weaker-than-ideal clippy evidence formatting.

**Refuted — close with no action:**
- `834`(b): the concern that `LoopGains` should carry `tau_s/theta_s/k_c_per_w/k_rpm_per_w/fitted_at`. The spec's authoritative struct at design line 507 is `loop_gains: Option<LoopGains>` with nothing else, and task 10's `derive_gains` (`src/calib/fopdt.rs:223`) already returns exactly the four gains. The reviewer's "fwloop.21 and fwloop.11 should confirm" resolves as: **no extension needed.**

---

## 5. Parked findings

**None.** `grep parked` over the ledger returns 0. No adjudicator overrode a finding to let a task merge in this run.

---

## 6. Untested scope (not findings)

**No `BLOCKED-AUTH` lines** — 0 occurrences. No task lost coverage to a harness permission refusal.

One adjacent gap worth recording under the same heading, since it is unmeasured scope rather than a defect:
- **CPU-axis `DEMAND_MARGIN_W` is unmeasured.** `9it`'s RESOLVE lays out the full procedure (unload `ryzen_smu` and *leave it unloaded*, then sample `PPT LIMIT SLOW` vs `PPT VALUE SLOW` under load at several caps). The agent's attempt #4 reloaded the module immediately, restoring the broken precondition. Never re-run. The spec's own acceptance requires this be *measured, not assumed*.
- **GPU-axis margin** was measured (1492 MHz against a 1500 MHz lock, 95–98% util, 7 ticks, ~0.11 W spread) and meets the spec's own §2.9 pin criterion, but was never written into §Facts, and the glxgears fill/geometry-bound-proxy limitation is unrecorded.
- **Never in the branch measurement at all:** `fw-fanctrl-loop-zct`, `fw-fanctrl-loop-dsh` — escalated, never merged. Their code is unreviewed against this tip.

---

## Recommended order to unblock

1. Re-open `fw-fanctrl-loop-9it` — it is closed against zero delivered artifacts (B2). Nothing downstream is trustworthy until §2.4 has a rule.
2. Fix the ±∞ `t_star` seam in `curve.rs`/`mode.rs` and get §2.1 the sign-off task 1 asked for (B1).
3. Add the `blocks` edge `24s → dsh` (and resolve `6ma.1`/`bwt` the same way) — the plan already states this ordering; beads never encoded it. This is the fix for the hot-file defect, retroactively.
4. Land `zct`, `dsh`, then `j6s`. The clippy sweep goes green as a side effect of `j6s`, and Auto mode stops being a floor-pinned no-op.
5. `eyi` sweeps Cluster B wholesale.
6. Re-run both gates. Only then is there a branch to land.
# Coverage ledger — fw-fanctrl closed loop (root slug `fwloop`)

Spec: `docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md`. Every disposed finding of
the super-design coverage loop, one line each, stable id, disposition (auto — non-interactive
disposition per the skill; the run is Mode A but no finding was a re-design GAP or an ORPHAN, so
no escalation fired). Findings are unioned across the three reviewers per round and deduped by
identity (type + subject).

## Round 1 (2026-09-08)

Reviewers: 3/3 valid (A, B, C). `requirements: 18 · mapped: 18 · unmapped: 0`.
R-new proposals appended for round 2: R19 target→duty snap pipeline; R20 warm-start vs bumpless
reconciliation; R21 `Released` release/re-engage path; R22 anti-windup against applied power;
R23 refinement convergence under table error; R24 on-machine validation.

| id | type | subject | disposition | note |
|---|---|---|---|---|
| C1-01 | UNOWNED-SEAM / GAP | target RPM → effective target → `target_duty` snap + no-tread fallback (A, B, C) | applied | fwloop.1 owns `Curve::nearest_tread`; fwloop.12 owns the snap and depends on fwloop.1; spec §1 amended |
| C1-02 | GAP / UNEXERCISED-CONFIGURATION | `LoopMode::Released` controller path and end-to-end run (A, B, C) | applied | §2.5 amended; fwloop.12 owns the branch; fwloop.17 gains the Released run and the `active:false` run |
| C1-03 | UNSATISFIABLE-ACCEPTANCE (unwired) / NARRATIVE-EDGE | fwloop.14 vs fwloop.13/15/11/18 removals (A, B, C) | applied | edges 14→13, 14→15 added; 11 already transitive via 12 and now names the `state.rs` fit-test deletion; README hits excluded from 14's criterion (owned by 18); `contour`/`CONSERVATIVE_START`/`overshoot_settle` added to the sweep list |
| C1-04 | UNSATISFIABLE-ACCEPTANCE / GAP | warm-start re-seed on key change contradicts bumpless; fwloop.17 not behind fwloop.19 (A, B) | applied | §2.4 decided: seed only on auto entry / re-engagement / calibration exit, never on a key change; fwloop.19 acceptance covers strategy, duty and AC re-keys with Δu = 0; edge 17→19 added |
| C1-05 | NARRATIVE-EDGE (unstated) | fwloop.15 renders calibration phase names owned by fwloop.13 (A, B) | applied | phase label decided as a plain string on `CalibProgressLite` (fwloop.8/13); no edge needed, stated in 15 |
| C1-06 | NARRATIVE-EDGE (unstated) | fwloop.18 documents flags owned by fwloop.8 (B) | applied | edge 18→8 added with blocked-by line |
| C1-07 | NARRATIVE-EDGE (unstated) | fwloop.8 `ec_argmax` typed by fwloop.3's `EcLabel` (A) | applied | field stated as `Option<String>` in fwloop.8 (matches §3.2) |
| C1-08 | UNOWNED-SEAM | `gpu_floor_w` / budget bounds derivation from the LUT (A, B, C) | applied | fwloop.12 owns bounds derivation and `set_bounds`; exposed as `CalibContext.budget_bounds` |
| C1-09 | UNOWNED-SEAM | `view_changed` producer (A, C) | applied | fwloop.9 owns `Sample.fanctrl_view_changed`; fwloop.12 passes it to the arbiter |
| C1-10 | GAP | `rpm_smoothed` / `fan_valid` source undefined (A, C) | applied | §1 names the existing `FAN_SMOOTH_N` tail-mean and sampler flag; fwloop.12 owns their supply |
| C1-11 | GAP | on-machine / real 30 min session validation (A, B) | rejected | the tree stops at merge-ready; field validation is user-owned post-merge (spec §6 records it); the in-tree half — perturbed-plant robustness — is applied under C1-13 |
| C1-12 | GAP | fixture corpus capture unowned (B, C) | applied | new leaf fwloop.20 `Capture machine fixtures`, blocker of 1, 2, 3, 7, 11, 16 |
| C1-13 | GAP | acceptance sim shares the controller's seeded table; no perturbation; refinement convergence untested (B, C) | applied | fwloop.16 `FanPlant` has its own offsettable table; fwloop.17 adds ±50 % K/τ/θ runs and the −8 % table-bias convergence run |
| C1-14 | UNOWNED-SEAM | fwloop.16 emulator emits no `FanctrlView`/`Freshness`/labelled EC channels/guard temps for fwloop.17 (B, C) | applied | fwloop.16 gains deps 2, 3 and emits a full `Sample` per tick incl. `view(now)`, freshness hook, ambient/charger channels, gpu/nvme tracks |
| C1-15 | GAP | flag-clear conditions and the 15 s `print speed` staleness rule untested (B) | applied | acceptance bullets added to fwloop.2, 9, 10 |
| C1-16 | GAP | integrator winds up against guard-overridden / slew-clamped applied power (C) | applied | §2.4 `observe_applied` back-calculation; fwloop.4 owns it, fwloop.12 reports the realised sum; acceptance in 4, 12, 17 |
| C1-17 | UNOWNED-SEAM | fwloop.12 and fwloop.13 both touch `controller.rs` with no edge (B) | rejected | fwloop.13's controller touch is a one-line call-site update with `CalibContext::default()`; execution's hot-file cap already serialises overlapping files, and an edge would idle 13 behind the hub for no artifact |
| C1-18 | UNSATISFIABLE-ACCEPTANCE (prose) | fwloop.5 "no reference in the crate" exceeds its file scope (C) | applied | criterion narrowed to `src/control/`; repo-wide terms moved to fwloop.14 |
| C1-19 | flag-sweep | fwloop.13 `sp:demoted-by-session` (A, B, C) | applied | demotion upheld (interface decided in §3.3); the reviewers' optional split accepted: `fit_fopdt` + IMC derivation moved to new leaf fwloop.21, blocker of 13; 13's dependents (14, 18, 19) re-checked — all still consume 13's own artifacts, unchanged |
| C1-20 | mechanical (6a) | `(needs: fwloop.12)` in fwloop.17 | no finding | direct blocker; passes (all three reviewers) |

## Round 2 (2026-09-08) — final round (cap)

Reviewers: 3/3 valid (A, B, C; C was dispatched late after only two of three launched, so the
pass ran at full strength). `requirements: 24 · mapped: 24 · unmapped: 0`. One R-new proposed
(C: unreachable-from-below), folded into R25 below and applied under C2-33.

**Divergence observation:** round 1 disposed 17 deduped findings; round 2 disposed 41, of which
38 are novel identities (3 re-litigate round-1 subjects: the ENODATA fixture convention, the
fwloop.12 size, hardware validation). The count did not shrink and the findings are mostly
novel — round 2 **widened** scope rather than converged. Under the fixed two-round cap its fixes
are applied below and are **not re-reviewed**; the root integration sweep (fwloop.23) is the
net for what this round's own fixes may have introduced. This is the strongest signal in the
run that the tree deserves a human read-through before execution; it is surfaced in the
hand-off summary rather than fixed by a third round.

| id | type | subject | disposition | note |
|---|---|---|---|---|
| C2-01 | UNOWNED-SEAM | fwloop.16 builds `Sample` without an edge to fwloop.9 (A, B, C) | applied | dep 16→9 |
| C2-02 | GAP | no `Budget` gains API; persisted gains never reach the integrator (A) | applied | `Budget::new(&LoopGains)`/`set_gains` in 4; 12 loads on auto entry |
| C2-03 | GAP | GPU `verify_lock` never wired (A) | applied | 12 runs both verdicts through one rule |
| C2-04 | GAP | three-strike release + same-tick reassert untested (A, C) | applied | 12 acceptance, 17 fault list |
| C2-05 | GAP | `gpu_*` sensors enter fw-fanctrl's max when the dGPU is powered; replica ignored them (A) | applied | §2.2 + §Facts amended: gpu_* included and controllable; second fixture tree (20); gpu channels in 16; dGPU-on run in 17 |
| C2-06 | GAP | curve cache keyed by strategy name misses in-place edits (A, C) | applied | 10 keys on points, `t_star_changed`; 16 `edit_curve_in_place`; 17 pins the t=900 edit |
| C2-07 | GAP | legacy state without `duty_rpm_table` must load the seed (A) | applied | `Default` = seed + serde default (1); 11 acceptance |
| C2-08 | UNOWNED-SEAM | poller construction from config / shutdown unowned (A) | applied | 9 owns construction in `main.rs` |
| C2-09 | UNOWNED-SEAM | `AutoAllocated.error` type would drag `LoopError` into fwloop.8 (A) | applied | `error: f64` |
| C2-10 | UNEXERCISED-CONFIGURATION | calibration → fitted gains → closed loop never run (A, B) | applied | 17 calibration run |
| C2-11 | GAP | `set_interval` retain-vs-clear unspecified (A) | applied | 3 contract + acceptance; 12 acceptance |
| C2-12 | GAP | burner / `NeedsLoad` missing from fwloop.13 (A, C) | applied | 13 owns them; no-rise rejection |
| C2-13 | GAP | `min(MA, current)` branch never graded (A) | applied | 16 acceptance; 17 load-release run |
| C2-14 | NARRATIVE-EDGE | fwloop.10 consumes token for 9 without edge (A) | applied | token reworded: plain bool from the controller |
| C2-15 | UNSATISFIABLE-ACCEPTANCE | fwloop.20 "matching §Facts" unverifiable (A) | applied | §Facts now records the curves; 20 asserts the truncation + steep-tread facts |
| C2-16 | ORPHAN | `FanctrlView.update_freq` unconsumed (A, B) | applied | dropped from the struct |
| C2-17 | UNOWNED-SEAM | test-support module layout unnamed (A) | applied | `src/test_support/{mod,fixtures,fakes,plant}.rs` owned by 20 |
| C2-18 | GAP | RAPL stickiness watchdog preservation unowned (A, B) | applied | 7 leaves `on_sample` untouched; 12 acceptance retains the test |
| C2-19 | GAP | `Unreadable`/`Unverifiable` policy undefined; `--info` failure is the expected state with `ryzen_smu` loaded (B, C) | applied | §2.9 non-events; `ReadbackBlind` after six; 7/8/12/15 amended |
| C2-20 | GAP | Mode B error target unsnapped (B) | applied | §2.4: `rpm_for_duty(target_duty)`; 12 acceptance |
| C2-21 | GAP | no `e_{k−1}` resync on T* re-derivation without a mode switch (B) | applied | `Budget::resync_error`; 10 emits `t_star_changed`; 12 calls it |
| C2-22 | UNOWNED-SEAM | ENODATA fixture marker convention (B, C) | applied | label without `_input`; reader drops unreadable inputs (3, 20) |
| C2-23 | GAP | clock seam for duration-shaped criteria (B) | applied | 9 owns the timestamps-not-wall-clock rule; §5 states it |
| C2-24 | GAP | socket read-only invariant untested (B) | applied | `PrintCommand` enum; fake records; 17 global assertion |
| C2-25 | GAP | refinement can make the table non-monotone (B) | applied | §2.3 clamp + 25 % rejection; 1 acceptance |
| C2-26 | UNOWNED-SEAM | 5/7/8/11 co-edit controller.rs; 5 deletes trim.rs 8 still references (B, C) | applied | call sites named per task; trim.rs deletion moved to 14 |
| C2-27 | GAP | integrator state during LutSweep unspecified (B) | applied | §3.3 whole-session `Calibrating`; 19 acceptance |
| C2-28 | GAP | RNG dependency unowned (B) | applied | hand-rolled xorshift in 16 |
| C2-29 | UNOWNED-SEAM | duplicate curve decoders / strategy resolution unowned (B, C) | applied | 2 owns `resolve_curve`; 1 drops `from_config` and tests on point lists (dep 1→20 dropped) |
| C2-30 | GAP | `print speed` 5 s cadence unasserted (B) | applied | 9 acceptance |
| C2-31 | GAP | fwloop.12 still oversized: destructive half separable (B) | applied | new leaf fwloop.22 (dep 8); 12 depends on 22; 14 re-pointed 12→22; 17/19 unchanged (§Splitting a Bead checked) |
| C2-32 | NARRATIVE-EDGE | fwloop.12 consumes `EcAverage` without an edge to 3 (B) | applied | dep 12→3 |
| C2-33 | GAP (R25) | target unreachable from below undetected (B, C) | applied | §2.7 low rule; `at_lower_bound_for`; 10 acceptance; 17 sub-floor run |
| C2-34 | GAP | fwloop.12 NVMe criterion inverted ("lowers T*") (C) | applied | corrected to raises T* and the budget; 17 too |
| C2-35 | GAP | reconciliation keyed on any-poll `observed_at` compares a stale temperature (C) | applied | two stamps in `FanctrlView`; `view_changed` keyed on `all_observed_at` (2, 9) |
| C2-36 | UNSATISFIABLE-ACCEPTANCE | fwloop.19 "Δu = 0" unsatisfiable while the PI runs (C) | applied | reworded: not re-seeded; ordinary increment |
| C2-37 | GAP | steady-window threshold vs ±90 RPM plant noise (C) | applied | detector on `rpm_smoothed`; 17 asserts ≥ 1 steady window per converged run |
| C2-38 | GAP | sample cadence vs boxcar sizing; `ec_ma` never checked against the emulator (C) | applied | 1 Hz stated (3, 9); 17 tracks within 1 °C |
| C2-39 | GAP | plant lacks utilisation / SM-clock channels for demand, GPU PI, `verify_lock` (C) | applied | 16 tracks; 17 load step is a utilisation step |
| C2-40 | UNOWNED-SEAM | `fitted_at` set by nobody (C) | applied | 13 stamps it; 21 leaves it `None` |
| C2-41 | UNEXERCISED-CONFIGURATION | dGPU-unpowered configuration; guards have no absent representation (C) | applied | `Option<f64>` guard inputs (6, §2.8); 17 dGPU-off run |
| C2-42 | mechanical (6a) | `(needs: fwloop.12)`, `(needs: fwloop.19)` in fwloop.17 | no finding | both direct blockers; pass (all three reviewers) |

**Root integration sweep:** fwloop.23 created after the loop ended, depending on every other
leaf with the fixed `all leaves (integration sweep)` token.

## super-roast design iteration 1 (2026-09-08)

Report: `2026-09-08-fw-fanctrl-loop-roast-design-1.md`. Verdict **Blocking (23 confirmed)**, no
qualifier: 8 scouts, 0 dead, 245 raw → 75 deduped, 57 panels + 18 spot checks, judge completion
100 %, 0 beyond the panel cap, 0 escalations, 34 low-severity candidates dropped by the
remainder cap. Severities: 1 Blocking, 19 Should-fix, 3 Nit.

**R1 (Blocking) — `observe_applied` was an identity.** It back-calculated against the commanded
caps, which `split_budget` guarantees sum to `u`, so it corrected nothing outside guard and slew
cases. The demand-starved wind-up it was supposed to prevent was wide open: at idle the
temperature error stays positive, the integrator saturates at the maxima, and the next load
onset runs uncapped through the dead time — with the old conservative start and model contour
deleted. Fixed by back-calculating against the **measured** smoothed draw (`cpu_pkg_w` +
`gpu_w`, both already sampled). §2.4, fwloop.4/9/12/16/17.

Applied, all 23: R1 above; R2 **held for the user** (NVMe guard remedy, see below); R3 defaults
re-derived with `θ_eff = θ + N/2`; R4 Mode B gain scheduled on `slope_at(T*)`; R5 fit rejection
gains magnitude and `Kc`-ratio bounds; R6 fit gain uses the measured per-axis delta and the
burner starts before the settle; R7 the unsatisfiable 25 % criterion restated against the
filtered plant; R8 `at_upper_bound_for` + `high` unreachable; R9 the moving average is
reconciled too and the emulator reproduces the paused-buffer and 50 °C quirks; R10 scored views
skipped while slewing or stale; R11 `EcAverage` seeded on every engagement, `unreconciled`
initial state; R12 argmax-controllable debounced; R13 `resumed` invalidates the boxcar and
windows; R14 refinement gated on `speed_pct == target_duty`; R15 relay test made
period-agnostic; R16 `gpu_hot_c` 83 → **90/85**, derived from the measured 87 °C card target;
R17 non-monotone curves rejected; R18 read-back keeps running after the three-strike release;
R19 mismatch re-read once and suppressed across `on_ac` edges; R20 StepTest gates in two stages
so it stops self-skipping, plus a 95 °C abort; R22 fwloop.8's acceptance narrowed to its own
type surface; R23 fwloop.5 added to fwloop.14's deps.

**Measurement-driven correction to round 2 (not a roast finding).** C2-05 amended §2.2 on a
reviewer's reasoning that the EC `gpu_*` sensors come alive when the dGPU is powered. Measured
2026-09-08 with the dGPU at 18.9 W in P0: they still read −150 and ENODATA. The original
research doc was right and the round-2 amendment was a regression. The replica keeps the
"every positive reading joins the max" rule (correct replication either way), the fixture set
now pins the measured fact, and the fabricated dGPU-powered acceptance run is gone. This is
exactly the class round 2's un-re-reviewed surface was expected to hide, and only a live
measurement could catch it.

**R2 — measured and applied 2026-09-08 (user decision: reporting-only).** The guard's remedy
raised the fan target, which in this cascade raises T* and therefore the power budget, on an
unmeasured assumption that airflow beats the extra heat. A probe settled it
(`docs/research/2026-09-08-nvme-airflow-probe.csv`, sustained O_DIRECT reads, aborted at an
82 °C safety cap after 50 s): at 4748 RPM the drive still went 66.9 → 79.9 °C in 30 s and kept
climbing, and the EC maximum *fell* 74 → 69 °C over the same window because the load was
I/O-bound and left the SoC idle. So the guard's lever is weak where it exists and absent in its
own main scenario. The run never plateaued, so it is not a clean low-versus-high airflow
comparison, and the spec says so. Resolution: the NVMe guard becomes **reporting-only** — flag,
status line and telemetry, no control action. `nvme_boost_rpm` and `Guards::effective_target`
are deleted, nothing modifies the user's RPM target any more, and fwloop.6/12/17/18 assert the
flag changes no commanded output. A follow-on in §6 records what a real lever would need.

**R21 — measured and applied 2026-09-08.** A live probe (fw-fanctrl paused, CPU load ramp, 300
samples of EC max against fan RPM) found the EC's own curve is a staircase that saturates early:
4096 RPM at 61–62 °C, 4520 at 63, 4658 at 64, then **flat at ~4748 RPM across 67–73 °C**, versus
2649 RPM under `quiet16` at a higher temperature. So under `active: false` the plant gain from
watts to RPM is ≈ 0 in the normal band and steep (140–420 RPM/°C) below 64 °C — neither
resembling the plant Mode B's gain was identified against. Resolution: do not fight it. §2.5 now
states that RpmLoop under `active: false` is expected to have little authority, the `low`
unreachable rule surfaces it as `TARGET UNREACHABLE (low)` with the achievable RPM, and the
applied-power back-calculation stops the integrator winding meanwhile. The plant gains an
EC-autofan mode carrying the measured staircase (fwloop.16) and fwloop.17 gains an authority run
covering both the flat and the steep segment.

## super-roast design iteration 2 (2026-09-08)

Report: `2026-09-08-fw-fanctrl-loop-roast-design-2.md`. Verdict **Blocking (12 confirmed)**, no
qualifier: 9 scouts (the `regression` lens added), 0 dead, 39 raw → 18 deduped, 17 panels + 1
spot check, judge completion 100 %, 0 beyond either cap, 0 escalations.
`delta vs prior: 7 new confirmed (2 Blocking) · 0 carried (0 Blocking) · 18 resolved · 5
regressed (0 Blocking)`.

**The loop's thrash exit fired** — Blocking went 1 → 2, which is not a shrink — so the run
paused for the user, who chose to apply everything and spend the third (capped) round.

**Both new Blocking findings were second-order damage from iteration 1's own R1 fix**, and both
are now fixed by replacing that mechanism rather than patching it:
- The back-calculation toward measured draw was **ungated**, so it stopped being anti-windup and
  became a tracker: equilibrium `u = draw + Kc·e`, meaning every lull dragged the cap onto the
  lull's consumption and starved the next onset for minutes while the integrator recharged at a
  few watts per minute.
- It compared the **combined sum**, so any structurally undrawn component (the GPU floor share
  with the dGPU unpowered) was a permanent gap the loop tried to close by lowering `u` until the
  CPU sat pinned at its floor — in a quadrant neither §2.7 rule names.

Resolution: **there is no back-calculation toward the draw at all.** §2.4 now specifies a
per-axis `Freeze::DemandLimited` — hold `u` when *every* axis draws more than `DEMAND_MARGIN_W`
below its own cap, keep integrating when any axis is at its cap. Clamping back-calculation
against the bounds is unchanged. That fixes the original wind-up without the collapse, and makes
the undrawn-share case inert.

Other 10 applied: R3 reconciliation scored at 1 Hz in the controller (the arbiter owns the
counters' meaning, not their sampling) plus an `unreconciled` reason; R4 `Decision.slope` is an
`Option` with a `0.25×` fallback when no curve is resolved; R5 Mode B's numeric defaults derived
and stated (`kc_w_per_rpm = 0.0028`, `ti_rpm_s = 35`); R6 the `θ_eff` substitution restricted to
the raw-domain defaults so the calibration fit stops counting the boxcar twice; R7 the two stale
`effective_target`/NVMe-boost references deleted; R8 `at_upper_bound_for` added to fwloop.10's
`ArbiterInput` with a `high`-reason acceptance case; R9 §2.9 no longer re-seeds the integrator on
a `Verified` (it calls `resync_error` and unfreezes, leaving `u` alone); R10 socket `Absent` and
`active: false` recognised as one plant regime (`ExecStopPost --autofanctrl`) with a graded run;
R11 the NVMe read moved off the 1 Hz sampler thread to the 30 s poller; R12 a rejected curve gets
its own warning-severity `CURVE INVALID` flag and a `curve_valid` arbiter input.

**Bonus validation from the same probe:** under `quiet16` at EC max 75 °C the curve gives duty 31
and the fans ran 2649 RPM against the seeded table's interpolated 2638 — under 0.5 % error, so
the shipped duty→RPM seed is sound and refinement really is a refinement rather than a
dependency.

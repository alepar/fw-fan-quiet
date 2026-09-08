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

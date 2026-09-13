# Final whole-epic review — run wf_710cf6d8-b1b (2026-09-09), THE COMPLETING RUN

stopReason: root-closed  |  completed: 25  |  escalated: []
parked: []  |  pendingRetry: []  |  stalled: False
authRefused: []  |  ledgerAppendFailed: []

## Metrics

    Metrics: merges 25 · merge-failed 2 · rebase-conflicts 1 · seam-reviews 9 (fixed 1) · gate-fails 2
    Metrics: fix-loop round 1: 6 addressed / 6 entered · round 2: 0 addressed / 0 entered · round 3: 0 addressed / 0 entered · round 4: 0 addressed / 0 entered · round 5: 0 addressed / 0 entered
    Metrics: fix-loop breaker-tripped: 0
    Metrics: ledger-check ok · append-failed 0 · append-retried 1

## Sweep

    07b31dd — 0 passed, 0 failed, 0 errors, 0 skipped; failing: none; command: cargo clippy --all-targets -- -D warnings Note: `cargo clippy` is a lint/build check, not a test runner — it produces a pass/fail build result (clean, exit code 0, no warnings or errors), not per-test counts. Full output confirms a clean build with zero lint diagnostics.

## Review

# Whole-epic review — `epic-fw-fanctrl-loop-6ma-integration` @ `07b31dd`

## Measurement at the tip (verified by me, not taken on trust)
- `cargo clippy --all-targets -- -D warnings` → **clean, exit 0**. Confirms the sweep line.
- `cargo test` → **630 passed, 0 failed, 3 ignored**. The configured branch-wide sweep is clippy-only, so the epic never had a branch-wide *test* measurement; this is it, and it is green.

## Ledger categories present
- `minor (deferred)`: **50** lines across 22 tasks.
- `parked`: **none**. `BLOCKED-AUTH`: **none**. `Recurring minor:` / `Recurring blocker:`: **none emitted** — yet the 50 minors cluster hard (below). The coordinator's cluster detector did not fire on a corpus that plainly meets its own ≥3-task threshold three times over; treat the absence of `Recurring:` lines as a detector gap, not as evidence of 50 independent nits.

---

## Clusters first — three classes, not 17 nits

**C1 — Assertions that cannot fail (8 instances / 8 tasks: blm, iym, dsh, 0nv, 438 ×2, cm7, nsc).**
Ledger lines 5, 46, 78, 89, 116, 118, 124, 129. Not a style issue: in four of these the unfailable assertion is the *only* guard on the acceptance criterion it was written for — `Freeze::Calibrating` wiring (116, asserts `budget_w == 0.0`, which holds whether or not the freeze exists), the no-reseed rule (118, passes on a 0-vs-0 delta), resume-clearing (124, outcome identical either way), and the whole `wiring_sweep` module (129, asserts literals against themselves). Root cause is pipeline, not people: implementers produce one test per acceptance sentence and satisfy it with whatever compiles; every reviewer caught it and every reviewer deferred it, so the loop never closed. Net effect: the 630-green is partly ornamental in precisely the spots the criteria called load-bearing.

**C2 — Stale doc comments left behind by the scope fences (5 instances / 5 tasks: fo1, 24s, 51b, mjv, 438).**
Ledger lines 18, 56, 62, 66, 117. Every one is a task correctly refusing to edit a file outside its `filesTouched` while removing the thing that file's comments describe (`StatusFlag::ModelDistrust`, "Empty stub here", "re-seeds from the floors", `flag_severity`'s NvmeHot severity). Correct process, wrong outcome — the plan's per-task ownership table has no owner for cross-file comment reconciliation. One sweep commit retires all five.

**C3 — `clippy -D warnings` red for most of the epic (4 instances / 4 tasks: 834, jpg, 24s, cm7).**
Ledger lines 8, 31, 57, 122. The launch config made `cargo test` the per-task gate and clippy an end-of-epic sweep, so accumulating `dead_code` from deliberately-unwired modules kept the lint gate meaningless mid-epic — and task 22 shipped **7 genuinely new lints** (needless_range_loop ×4, type_complexity, ptr_arg, too_many_arguments) under cover of that pre-existing breakage, with the report calling plain `cargo clippy` "clean". **Resolved at the tip** (verified above). No action on the code; the lesson is that a gate which is knowingly red is not a gate.

---

## Must be addressed before this branch lands

**1. `fw-fanctrl-loop-a78` — GPU_TRIP_C (87 °C) sits below GPU_HOT_C_DEFAULT (90 °C). BLOCKING.**
`src/control/watchdog.rs:19` = 87.0; `src/control/guards.rs:32` = 90.0. Verified live. Two consequences, both shipping: (a) three consecutive seconds at the card's own documented normal sustained-load parking point latches `ThermalEmergency`, releases every limit, and demands a manual re-arm — a routine gaming session, not a fault; (b) `GuardState.gpu_hot` and `gpu_share_override` are unreachable in Auto mode at any temperature, so an entire subsystem (task 6's whole deliverable) is dead in production. Its regression test is `#[ignore]`d at `src/control/sim_tests.rs:2317`, and the design doc (§ around line 1006) *explicitly* declines to fix it inline and defers it as a filed blocker. This is the one item that is a product defect rather than a hygiene debt, and the epic deliberately routed around it. It needs the tuning decision (raise GPU_TRIP_C above 90, mirroring CPU_TRIP_C=95 sitting above its soft point) and the test un-ignored.

**2. Ledger line 116 — make the `Freeze::Calibrating` test able to fail.** Given C1, at least this one instance must become real before landing: the freeze *is* correctly wired (`calib_budget.step(..., Some(BudgetFreeze::Calibrating))`, verified by reading `controller.rs`), but nothing detects its removal. Deleting the wiring keeps the suite green. Same argument applies with less force to 118 and 124, which were at least mutation-verified against their actual finding.

**3. Close two stale beads before the branch is judged.** `bd list` shows three open, two of which no longer describe reality and misrepresent the branch:
- `fw-fanctrl-loop-7e9` ("§2.4 anti-windup rule not rewritten") — **stale**: §2.4 *was* rewritten in commit `7941481`; the "Open for the spike to decide" paragraph is gone, `DEMAND_MARGIN_W_CPU = 2.0` / `_GPU = 3.0` are measured and recorded in §Facts, `ConditionalHysteresis` is normative with the sweep table at §2.4.
- `fw-fanctrl-loop-bwt` (PersistedState schema breaks ~50 tests) — **stale**: dsh landed, 24s landed, suite is green.
- `fw-fanctrl-loop-a78` — **genuinely open**, see item 1.

## Should fix before landing (cheap, none blocking on their own)
- **Ledger 94** — `nalgebra = "0.33"` still in `Cargo.toml:15` with zero `.rs` importers left after kalman.rs/thermal_model.rs were deleted. Verified: `grep -rl nalgebra src/` returns nothing. One-line delete.
- **C2's five stale comments** — one sweep commit, now that no scope fence applies.
- **Ledger 88** — dead cruft `let entry_effects_idx = 0; ... let _ = entry_effects_idx;` in `src/calib/runner.rs`.

## Safe to defer (with reasons, so they aren't re-litigated)
- **Already retired by later work**, do not action: line 15 (tread ±INFINITY endpoints — fixed by task 25, commit `673002c`); line 126 (the a5j-blocked bundled assertion — a5j was fixed in task 24; only 2 hardware-gated `#[ignore]`s plus a78 remain); C3's four clippy entries.
- **Line 11** (no `resync_error` on a direct Freeze→Freeze transition) — I traced it: reachable in principle (`Released`→`DemandLimited`, since `seed()` doesn't touch `e_prev` and the hard-hold path returns before updating it), but the `ActuatorMismatch` exit is explicitly resynced at `controller.rs:2105`, and the residual case is a one-tick, downward-only, bound-clamped over-decrement. Real, bounded, not a lander.
- **Line 106** (the anti-windup spike drove `split_budget`, not the rate-limited `Allocator::step`) — the strongest-looking design objection in the ledger, and it is now retired by evidence rather than argument: task 22's closed-loop sims exercise the same conclusion through the *production* path (`demand_starved_idle_never_winds_u_to_the_upper_bound...`, `duty_cycled_load_does_not_let_u_decay...`, both passing).
- Lines 10, 40, 47, 52, 110–112, 125, 130 — genuine but small (mutex-poison cascade, `dedup()` adjacency invariant, one-tick cap lag, a redundant `serde_json::from_str` per poll). File them; don't hold the branch.

## Untested scope (report, not findings)
No `BLOCKED-AUTH` lines, but the analogous gap exists: **3 `#[ignore]`d tests never execute here** — `src/actuators/gpu.rs:386` and `src/sensors/gpu.rs:91` require a physical NVIDIA GPU + driver, and `src/control/sim_tests.rs:2317` is a78's blocked regression test. The two hardware ones mean the real NVML actuator/sensor paths are covered only by fakes on this branch.

## Verdict
**Land after item 1.** The engineering is in good shape — clippy clean, 630/630 green, §2.4 settled and measured, the a5j defect found and fixed by the epic's own integration sweep. What must not ship is a78: the epic knowingly deferred a P1 thermal-tuning defect that makes a normal gaming session trip a latched emergency and renders the GPU soft guard unreachable, and marked its only detector `#[ignore]`. Items 2–3 are small and belong in the same commit; the C1 assertion-discipline class is the one thing worth changing about the *pipeline* rather than the branch.

Key paths: `/var/home/alepar/AleCode/fw-fan-quiet/.claude/worktrees/fw-fanctrl-loop/.worktrees/epic-fw-fanctrl-loop-6ma-integration/` — `.superpowers/sdd/fw-fanctrl-loop-6ma-plan/progress.md`, `src/control/watchdog.rs:19`, `src/control/guards.rs:32`, `src/control/sim_tests.rs:2317`, `src/control/controller.rs:5701`, `Cargo.toml:15`.
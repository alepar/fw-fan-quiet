## Task 12: Remove the adaptation tier

**Bead:** `fw-fanctrl-loop-24s`

**filesTouched:** `src/control/controller.rs`

### Global constraints

All of "Global Constraints" above applies. Normative: **§3.2**, §4.

This task owns `src/control/controller.rs` **as a whole** for the duration — it is the file's
demolition pass, and Tasks 19 and 20 build on what it leaves. Every other controller-touching
task before it was restricted to named call sites precisely so this one can be a clean deletion.

### What to delete

The five-gate adaptation tier; the cooldown ring; the trust monitor; the `ModelSnapshot` period;
the degrade guard's `model.is_none()` check; the `persisted_bias` / `persisted_gain` plumbing;
and **every controller test that exercises them** — the KF, trust, cooldown and `fitted_model`
blocks.

**Do not delete the module files themselves** (`thermal_model.rs`, `kalman.rs`, `trust.rs`,
`cooldown.rs`, `trim.rs`) — Task 21 (`fw-fanctrl-loop-eyi`) owns that, and is blocked on this
task for it. Your job is to remove the last importer inside `controller.rs`.

After this task the auto path compiles with the budget stubbed at the floors and the existing
`AllocInput` call site (from Task 6).

### Acceptance criteria (verbatim from the bead)

> no import of `thermal_model`, `kalman`, `trust`, `cooldown` remains in the controller (source
> or tests); no `adapt_bias`/`adapt_gain`/`persisted_bias` symbol remains; `cargo test` green.

### Implementation steps

1. Inventory first: list every symbol and test block in `controller.rs` that belongs to the
   tier. Put the list in your report — it is what the reviewer checks the deletion against.
2. Delete the tier's production code, then its tests, in that order, so the compiler names any
   test you missed.
3. Remove the `model.is_none()` branch from the degrade guard and the `persisted_bias` /
   `persisted_gain` plumbing.
4. Confirm the surviving auto path still compiles against the floors-stubbed budget and the
   Task 6 `AllocInput` call site.
5. **Verify:** search `src/control/controller.rs` for `thermal_model`, `kalman`, `trust`,
   `cooldown`, `adapt_bias`, `adapt_gain`, `persisted_bias` — zero hits, in source and in tests.
   Record the search output in your report.
6. Run the test suite and the linter; both clean.

### Deliverable

A tier-free `controller.rs` with the full surviving suite green, and the five doomed modules
still on disk for Task 21.

---


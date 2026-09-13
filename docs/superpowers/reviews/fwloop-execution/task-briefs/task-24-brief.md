## Task 24: Integration sweep: fw-fanctrl closed loop

**Bead:** `fw-fanctrl-loop-nsc`

**filesTouched:** `src/integration_tests.rs`, `src/main.rs`,
`docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md`

`src/main.rs` — the `#[cfg(test)] mod integration_tests;` declaration, plus any small inline fix
the sweep turns up. The design doc — the **Post-Implementation Notes** section only.

This task runs last, on the merged tree, and is the epic's root integration sweep. Its
`filesTouched` deliberately understates the blast radius: **small inline fixes may land anywhere
under `src/`**. Keep each one small; anything larger becomes a filed bead, not a diff here.

### Global constraints

All of "Global Constraints" above applies. Normative: the whole design doc.

### The three jobs

**(1) Walk the goal's main flows end to end and implement what is missing.**

- Engage auto from a **fresh `state.json` with only a LUT**.
- Walk `TempLoop` -> socket death -> `RpmLoop` -> recovery -> `TempLoop`.
- Run a calibration.
- Restart the daemon and confirm the **warm-start, table and gains reload**.

**(2) Sweep for unwired config values, parameters and interfaces.** Every `Config` key is read
somewhere; every `StatusFlag` is raised somewhere **and** rendered; every telemetry field is
populated; every `Effect` variant is applied; every `CalibContext` field originates from live
data. This is the sweep that catches a key that was added and never consulted.

**(3) Add the integration tests no per-task test covers:** sampler -> controller -> telemetry
line with **real** types; config -> poller construction; a full `on_command` / `on_sample`
session on the fakes.

**Fix small gaps inline; file a blocker bead for large ones.** Do not absorb a large gap into
this task quietly.

### Acceptance criteria (verbatim from the bead)

> the three flows above pass on the fakes; the unwired-sweep checklist is recorded in the spec's
> Post-Implementation Notes with zero open items or a filed blocker per item; `cargo test` and
> `cargo clippy -D warnings` green.

### Implementation steps

1. Create `src/integration_tests.rs` (`cfg(test)`) and declare it in `src/main.rs`.
2. **Test first:** flow 1 — auto entry from a LUT-only `state.json`, the A -> B -> A walk, a
   calibration, then a restart that reloads warm-start, table and gains. Implement whatever is
   missing to make it pass.
3. **Test first:** flow 3's three integration tests — sampler to telemetry line with real types;
   config to poller construction; a full `on_command`/`on_sample` session on the fakes.
4. Build the **unwired-sweep checklist** as five explicit enumerations: every `Config` key,
   every `StatusFlag`, every telemetry field, every `Effect` variant, every `CalibContext`
   field. For each, show where it is read/raised/rendered/populated/applied. Prefer writing each
   enumeration as a **test** over writing it as prose, so it cannot rot.
5. Fix small gaps inline. For each large one, file a bead (`bd create`) and record its id.
6. Record the completed checklist in the spec's **Post-Implementation Notes**, with **zero open
   items** or a filed blocker id per item.
7. Run the test suite and the linter; both clean.

### Deliverable

A merged tree whose main flows are proven end to end, with a recorded sweep checklist that has
no open item without a bead id attached.

---


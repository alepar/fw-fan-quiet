## Task 21: Deletion sweep

**Bead:** `fw-fanctrl-loop-eyi`

**filesTouched:** `src/control/thermal_model.rs`, `src/control/kalman.rs`,
`src/control/trust.rs`, `src/control/cooldown.rs`, `src/control/trim.rs`, `src/control/mod.rs`,
`src/control/gpu_pid.rs`, `TODO.md`

The five module files are **deleted**. `src/control/mod.rs` loses their `pub mod` lines.
`gpu_pid.rs` and `TODO.md` lose stray references.

### Global constraints

All of "Global Constraints" above applies. Normative: **§4** (deletions — "must sweep the whole
repo, source and non-source").

By the time this task runs, the importers are already gone: the controller's tier and tests with
Task 12, the `state.rs` fit tests with Task 13, the matrix code with Task 18, the telemetry
fields with Task 15, and `trim.rs`'s last user with Task 6. **This task removes whatever is
left** — stray imports, comments, docs and TODO mentions.

`README.md` is **not** yours — Task 23 (`fw-fanctrl-loop-7ij`) owns it.

### Acceptance criteria (verbatim from the bead)

> a repo-wide search for `thermal_model`, `kalman`, `trust::`, `cooldown`, `trim::`,
> `adapt_bias`, `ModelSnapshot`, `trim_rpm`, `contour`, `CONSERVATIVE_START`, `overshoot_settle`
> hits only `docs/research/`, `docs/plans/` history, this spec, and `README.md` (owned by
> fwloop.18); no `src/` hit remains; `cargo test` and `cargo clippy -D warnings` green.

### Implementation steps

1. **Search first, and record the full "before" output in your report.** Search the whole repo —
   source **and** non-source (docs, `TODO.md`, `README.md`, manifests) — for each of the eleven
   strings above **and** for each of the five module basenames.
2. Classify every hit: delete here, owned by Task 23 (`README.md`), or legitimately historical
   (`docs/research/`, `docs/plans/`, the spec itself).
3. Delete the five module files and their `pub mod` lines in `src/control/mod.rs`.
4. Remove the remaining references — `gpu_pid.rs` comments, stray imports, `TODO.md` mentions.
5. **Search again** and confirm the "after" output hits only `docs/research/`, `docs/plans/`,
   the spec, and `README.md`. **Zero `src/` hits.** Record the output in your report.
6. Run the test suite and the linter; both clean.

### Deliverable

A repo with no deleted-subsystem residue outside history docs and the README, with before/after
search output in the report.

---


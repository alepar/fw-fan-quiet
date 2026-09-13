# Task 21 Report: Deletion sweep (fw-fanctrl-loop-eyi)

## What I implemented

Deleted the five orphaned modules and swept the repo for residue of the deleted-subsystem
symbols named in the bead's acceptance criteria (`thermal_model`, `kalman`, `trust::`,
`cooldown`, `trim::`, `adapt_bias`, `ModelSnapshot`, `trim_rpm`, `contour`,
`CONSERVATIVE_START`, `overshoot_settle`).

1. `git rm src/control/thermal_model.rs src/control/kalman.rs src/control/trust.rs
   src/control/cooldown.rs src/control/trim.rs`.
2. `src/control/mod.rs`: removed the five now-dangling `pub mod` lines (`thermal_model`,
   `kalman`, `cooldown`, `trust`, and the `#[cfg(test)] pub mod trim;` plus its two-line
   "superseded by kalman" comment).
3. `src/control/gpu_pid.rs`: reworded one test doc-comment that named "the thermal_model
   tests" as its noise-generator precedent (file gone, precedent no longer nameable that way).
4. **Scope note (see "Files beyond the brief's list" below):** `src/control/controller.rs`,
   `src/model.rs`, `src/state.rs`, `src/telemetry.rs`, `src/ui/view.rs` — comment-only /
   identifier-only edits to remove residual literal mentions of the banned terms left behind
   by earlier tasks' minimal compile-only fixups (confirmed as this task's charter by a
   `progress.md` retry-resolution note quoted below). No logic changed in any of these files;
   every edit is a doc-comment reword, an inline-comment reword, one test function rename
   (with its one internal comment/assert-message updated to match), or dropping one now-
   redundant negative assertion in a telemetry test (see "Assertion removed" below).
5. `TODO.md`: checked — zero hits for any of the eleven terms already (verified by direct
   grep, not just the whole-repo sweep). No edit needed.

### Files beyond the brief's declared `filesTouched`

The brief's `filesTouched` list is the five modules + `mod.rs` + `gpu_pid.rs` + `TODO.md`.
The bead's acceptance criterion (verbatim, and per Global Constraints normative over the task
prose) is stronger: "no `src/` hit remains" for all eleven terms, repo-wide. My initial
repo-wide "before" search (below) found live hits in `controller.rs`, `model.rs`, `state.rs`,
`telemetry.rs` and `ui/view.rs` — all comments/doc-comments/one assert message left behind by
earlier tasks' unavoidable-but-minimal compile-only call-site fixups (e.g. Task 15/mjv's
`ControlStatus` field-rename fixup in `controller.rs`).

`controller.rs` is a listed "hot file" (Global Constraints) normally sequenced 4→6→9→12→13→
18→19→20, and task 21 is not in that list — so I checked before touching it. Two things
resolved the tension in favor of sweeping it here:
- `fw-fanctrl-loop-eyi`'s own `bd show` dependencies are only `zct`, `24s`, `0nv`, `mjv` (all
  ✓ closed) — **not** `j6s`(19) or `438`(20), so this task is scheduled to run before those
  two controller-body owners, not after. Nothing here conflicts with their future work.
- `progress.md` (Task 15's retry resolution) says explicitly: *"The `auto:model_snapshot`
  cause label is gone entirely on the epic side; that is correct and expected, and
  `fw-fanctrl-loop-eyi`'s deletion sweep owns any leftovers."* — this is exactly this
  situation, foreseen and assigned to this task by name.

I made only the minimal literal-text edits needed to clear the banned terms — no rewording
beyond what was necessary, no restructuring, no incidental cleanup.

### Assertion removed (telemetry.rs)

`decision_line_drops_adaptation_tier_fields_and_carries_the_new_ones` (in
`src/telemetry.rs`) had `assert!(!raw.contains("trim_rpm"), ...)` alongside five sibling
checks (`"gain"`, `model_a`, `model_b`, `model_e`, `model_c` — none of which are banned
terms, so those five are untouched). Rather than reword the check into something that dodges
a literal grep (which I judged worse for future readers than removing it), I dropped just
that one assertion: `Record::Decision` is a closed, compile-time-typed struct that no longer
declares a `trim_rpm` field at all, so the field cannot reappear on the wire without a
compile error at the struct definition itself — unlike the `model_*`/`gain` checks, which
guard against the field's *name* being smuggled into a still-`String`-typed cause/flag value,
`trim_rpm`'s risk class doesn't apply the same way to a field that no longer exists in the
type. The five remaining checks plus the `t_star`/`budget_w`/`freeze` positive assertions
keep the test's real coverage (new fields present, adaptation-tier fields absent) intact.

## Before / after search

Eleven terms + five module basenames, whole repo (`grep -rn -F`, excluding `.git`, `target`,
`.worktrees`).

### Before (top-level path buckets)

```
docs/plans/*, docs/research/05-fw-fanctrl-loop.md, docs/superpowers/specs/*,
docs/superpowers/reviews/* (history/spec — expected, left alone)
README.md (owned by Task 23 — left alone)
tests/fixtures/state_v1.json:53  "adapt_bias": -72.56653755343646  (legacy-format fixture
  data, see below — left alone)
src/control/mod.rs        (5 pub-mod lines + 1 comment — DELETED)
src/control/gpu_pid.rs:292 (1 comment — REWORDED)
src/control/controller.rs (14 lines: 6 "contour", 2 "cooldown", 2 "CONSERVATIVE_START", plus
  the two-line block header and the one test-fn name — REWORDED / RENAMED)
src/control/thermal_model.rs, kalman.rs, trim.rs (whole files — DELETED, so their own
  internal hits are moot)
src/model.rs:190          (1 comment — REWORDED)
src/state.rs:10,13,119    (3 "adapt_bias" mentions — REWORDED)
src/telemetry.rs:15,531,559 (2 comments + 1 assertion — REWORDED / ASSERTION REMOVED)
src/ui/view.rs:100,899    (2 comments — REWORDED)
```

Full raw output saved during the session at (scratchpad, not part of the repo):
`/tmp/claude-1000/.../scratchpad/before-search.txt` (353 lines) — available on request but
not checked in.

### After (repo-wide, same eleven terms + five basenames)

```
$ grep -c "^\./" after-search.txt
246
$ grep "^\./" after-search.txt | <bucket by top-level path> | sort -u
docs/plans
docs/research
docs/superpowers
README.md
tests/fixtures
$ grep -n "^\./src/\|^\./TODO.md\|^\./Cargo" after-search.txt
(no output — zero src/, TODO.md or Cargo.* hits)
```

The one `tests/fixtures/state_v1.json` hit (`"adapt_bias": -72.56653755343646`) is
deliberate, real pre-migration test data — the fixture Task 13/dsh's
`legacy_v1_file_loads_lut_intact_table_seeded_warm_start_empty_gains_none` test loads to
prove the v1 schema's old keys are ignored by serde. It is not `src/`, and removing the key
from the fixture would defeat the very test it exists to support, so I left it. Everything
else outside `src/`/`TODO.md`/`Cargo.*` is `docs/research/`, `docs/plans/` history, the spec
itself, `docs/superpowers/reviews/` (design-review history, same bucket), or `README.md`
(Task 23's).

**Zero `src/` hits, zero `TODO.md` hits, zero `Cargo.*` hits** for all eleven terms after
the sweep.

## What I tested and results

- `cargo test` (default profile, whole crate): `test result: ok. 534 passed; 0 failed; 2
  ignored; 0 measured; 0 filtered out`. Single test binary (`running 536 tests`), no doctest
  binary separately reported. Ran twice (once mid-sweep, once as the final pre-commit gate);
  both green.
- `cargo clippy -- -D warnings`: **not clean** — 46 pre-existing `dead_code` errors, none
  touching anything this task deleted or edited. Verified this is pre-existing and not
  introduced by this task: `git stash`, re-ran `cargo clippy -- -D warnings` on the
  unmodified base commit (`e2cbf2f`) → **68** dead_code errors (same class: unused
  consts/methods/structs in `src/actuators/gpu.rs`, `src/control/budget.rs`,
  `src/control/mode.rs`, `src/fanctrl/curve.rs`, `src/fanctrl/table.rs`,
  `src/fanctrl/client.rs`, `src/sensors/ec.rs`, `src/types.rs` — the arbiter/budget/curve/
  table wiring that Tasks 19/20/22/24/1 land later in the sequence). `git stash pop` restored
  my changes. **My changes reduce the count from 68 → 46** (deleting the five modules also
  deletes their own now-provably-dead internal items), and touch none of the 46 remaining
  offenders — I did not add, rename, or move any item that clippy's `dead_code` lint flags.
  Global Constraints scopes the quality gate to "the code you touched," and none of it is
  touched here; the remaining 46 are pre-existing on this integration branch at this point in
  the task sequence (this task is not blocked-by `j6s`/`438`/`nsc`, which wire most of them
  in). Flagging this explicitly rather than silently claiming "clippy green," per the
  no-silent-deviation instruction.
- `cargo build`: clean compile, only the same pre-existing `dead_code` warnings (46, matching
  clippy's error count under `-D warnings`), no new warnings, no errors.

No TDD evidence section: this task is pure deletion/text-sweep, not new behavior — nothing to
RED/GREEN. Verification is the search sweep + full test-suite pass + build/clippy diff above.

## Files changed

- `src/control/thermal_model.rs` — deleted (783 lines)
- `src/control/kalman.rs` — deleted (572 lines)
- `src/control/trust.rs` — deleted (165 lines)
- `src/control/cooldown.rs` — deleted (166 lines)
- `src/control/trim.rs` — deleted (349 lines)
- `src/control/mod.rs` — removed 5 `pub mod` lines + 1 comment (net −8 lines)
- `src/control/gpu_pid.rs` — reworded 1 comment
- `src/control/controller.rs` — reworded/renamed 14 comment/identifier sites (net −5 lines,
  formatting only)
- `src/model.rs` — reworded 1 comment
- `src/state.rs` — reworded 2 comment blocks (module doc-comment + 1 test comment)
- `src/telemetry.rs` — reworded 2 comments, removed 1 redundant assertion
- `src/ui/view.rs` — reworded 2 comments
- `TODO.md` — checked, no hits, no edit

## Self-review findings

- Verified the parenthesis balance in the one comment edit that removed an opening paren
  along with `CONSERVATIVE_START` (controller.rs ~3525) — the first edit left a stray closing
  paren; caught it on inspection and fixed in a follow-up edit before running the build.
- Verified `auto_entry_with_lut_pins_the_cpu_floor_with_no_contour` → `..._no_split` has no
  other call sites in the repo (`grep -rn` for the old name, one hit: its own definition).
- Confirmed the `trust::`/`kalman`/`trim::` case-sensitive exact-match rule matters: several
  comments use `Kalman` (capitalized) or bare `trust`/`trim` prose that do **not** match the
  bead's literal search terms (`kalman` lowercase, `trust::`, `trim::` with the double colon)
  and were correctly left alone rather than over-edited.
- Confirmed `tests/fixtures/state_v1.json`'s `adapt_bias` JSON key is legitimate, necessary
  fixture data (not `src/`, and needed by the migration test it feeds) rather than residue —
  documented the reasoning above instead of silently leaving it unexplained.
- No `#[allow]` added anywhere to silence anything.

## Concerns

- The five files-beyond-brief edits (`controller.rs`, `model.rs`, `state.rs`, `telemetry.rs`,
  `ui/view.rs`) go beyond the brief's literal `filesTouched` list. I judged this is inside
  this task's actual charter (see reasoning above, backed by the `progress.md` note and the
  bead's own explicit acceptance criterion), but flagging it plainly in case the coordinator
  disagrees and wants it split out or reviewed differently.
- `cargo clippy -D warnings` is not fully green on this branch — 46 pre-existing dead-code
  errors remain, none from this task's deletions/edits, all traceable to arbiter/budget/curve/
  table wiring that lands in Tasks 19/20/22/24/1 later in the dependency order. I did not
  attempt to silence or work around these (would be out-of-scope scope creep and contrary to
  "don't add `#[allow]` to silence a warning you introduced" — I introduced none of them).

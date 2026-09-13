# fw-fanctrl-loop-6ma: a false `alreadyMerged` closed the epic's headline bead with zero work done, and a missing `schema:` zeroed the entire Metrics block — neither catchable by the dryRun harness

Filed: https://github.com/alepar/superpowers/issues/7 (2026-09-09, label `upstream-feedback`, unscrubbed)

Plugin: `superpowers-alepar/superpowers` 6.3.0-alepar3.8 (`skills/super-code`). Run: 25 beads, 3 coordinator invocations (1 productive round each), ~351 agents, ~24.5M subagent tokens, ~11 h wall, `stopReason: root-closed`, 2026-09-09. Tracker `bd` 1.2.2. Project is a Rust binary checked out as nested git worktrees.

Adaptation check: the executed script differs from the canonical skeleton by 91 lines in 10 hunks — two module constants (`SDD_DIR`/`SC_DIR`), one added `schema:`, and prompt text. No control flow, scheduler, phase, or schema logic differs. Every defect below is verified present in the canonical skeleton.

An independent fresh-context analyst (opus) produced the findings; the invoking session triaged them and confirmed four corrections the analyst made to the session's own friction log (noted inline where they matter).

## Defects

### 1. Known Limitation 3 fired live: a false `alreadyMerged` closed the epic's most important bead with zero work done
- **Evidence:** `Task 11 (fw-fanctrl-loop-9it): complete (already merged into epic-fw-fanctrl-loop-6ma-integration before this re-entry — bead closed, no new review)`. The bead is the §2.4 anti-windup **spike** — the one the operator's invocation singled out as "do not let an implementer invent that rule from prose". Verified absent at the tip: the spike file did not exist on any branch (`git log --all -- <path>` empty), the design doc's "Open for the spike to decide" paragraph was still verbatim, no measurement in §Facts. Sequence: run 1's implementer hit a hardware precondition and reverted its own changes → triage RESOLVE with a correct procedure → the same-round retry re-entered at the brief stage → brief agent answered `alreadyMerged: true` → `closeOnlyPrompt` ran `bd close`. The branch sat at exactly an integration commit with **zero commits of its own**; `taskBriefPrompt`'s own rule says that is NOT merged. Cost: a closed bead is invisible to `bd ready` forever, so run 2 drained at 21/24 with the headline deliverable absent (`fw-fanctrl-loop-7e9` filed by the correctly-stopping `j6s` implementer); the bead was reopened by hand; run 3 (81 agents, 7.0M tokens, 6.7 h) re-did it. A silent false-close is strictly worse than a blocker bead — it bypasses the escalation currency entirely.
- **Premise to verify:** `closeOnlyPrompt`'s agent already runs in the integration worktree with git available (it runs `bd close` there).
- **Suggested fix shape:** have `closeOnlyPrompt` re-run the second-parent check itself and report `merged: false` instead of closing when the branch carries no commits of its own — one unverifiable boolean becomes a two-agent agreement without giving the coordinator shell access.

### 2. `read-ledger:finish` is dispatched without `schema: LEDGER_TEXT`; the whole Finish Metrics block reads zero, and the dryRun harness cannot catch it by construction
- **Evidence:** Resume-phase dispatch is `{ label: 'read-ledger', phase: 'Resume', schema: LEDGER_TEXT, … }`; the Finish-phase one is `{ label: 'read-ledger:finish', phase: 'Finish', model: … }` — no `schema`. Without one the runtime returns a bare string, so `metricsLedger?.text` is `undefined` and `(undefined || '')` parses zero lines. Run 1 emitted `Metrics: merges 0 · merge-failed 0 · rebase-conflicts 0 · seam-reviews 0 (fixed 0) · gate-fails 0`, all fix-loop rounds `0 addressed / 0 entered`, and `ledger-check M≠completed: 0 vs 15` — against a ledger body carrying 16 `Merge:` lines, 1 rebase conflict, 2 `gate fail → blocker`, 9 seam reviews and 2 fix rounds. Journal inspection: only ONE result in the run carries a `text` key (the Resume read). With the one-line fix, runs 2 and 3 emitted `merges 20` / `merges 25` correctly. The blind spot: `pick()` returns the declared stub object before `dispatch()` ever sees `opts`, so a `{text: '…'}` stub makes every dryRun assertion pass on a script that cannot work live. **No assertion over stubbed dispatches can catch a missing-schema defect.**
- **Premise to verify:** the Workflow runtime returns a bare string (not `{text}`) when no schema is declared — verified here by journal, but runtime-version-dependent.
- **Suggested fix shape:** add the schema; and state in the dryRun policy that schema declarations are outside its coverage, or lint for dispatches that read a structured field off a schema-less result.

### 3. `mergePrompt` is the only blocker-filing site that does not restate the bare-`blocker`-label rule — and the only site that produced malformed beads (2 of 2)
- **Evidence:** four sites spell the rule out inline (`implementPrompt`, `missingBlockerBeadPrompt`, `unplannedBlockerPrompt`, `breakerBlockerPrompt`): *"run `bd create` with ONLY the `blocker` label — no `sp:` label, no other label, and no `--parent`"*. `mergePrompt` says only *"file a blocker bead (see "The blocker-bead path")"* — a cross-reference to a doc section the agent never reads. Tracker state via `bd show --json`:

  | bead | filed by | parent | labels | dependencies |
  |---|---|---|---|---|
  | `fw-fanctrl-loop-6ma.1` | merge gate (zct red gate) | epic | none | 24s, eyi, epic |
  | `fw-fanctrl-loop-6ma.2` | merge gate (mjv rebase) | epic | none | epic, mjv |
  | `bwt`, `7e9`, `2ll`, `wwh` | restated sites | none | `['blocker']` | none |

  2/2 malformed at the un-restated site, 0/4 at restated ones. Both became epic children reachable as work, entangled in the graph, and would have blocked epic closure permanently; `6ma.1` needed `bd close --force` because of a dependency it should never have had. (Corrects the invoking session's own friction log, which had misattributed `7e9` — correctly formed — and missed `6ma.2`.)
- **Premise to verify:** the dotted `.1`/`.2` ids were minted by `bd create --parent`.
- **Suggested fix shape:** make the label-only sentence a shared function and interpolate it into `mergePrompt` too; add a post-filing verification (the coordinator holds `RESULT.blockerBead`). Note `bd update <id> --label` does not exist in bd 1.2.2, so post-hoc repair is not a one-liner — prevention at filing time is the only cheap path.

### 4. `treeMembershipTest` is written against a `bd show --json` schema that bd 1.2.2 does not emit; both call sites break at once
- **Evidence:** the canonical test looks for a `dependencies` entry with `dependency_type: "parent-child"`. On bd 1.2.2 `bd show --json`'s entries carry only `id`/`title`/`description` and list **blocking** deps; the parent link is a **top-level `parent` field** (`fw-fanctrl-loop-9it` → `parent: "fw-fanctrl-loop-6ma"`; root → `null`). The bulk dump `bd list --json` DOES carry a `type` field (`blocks` / `parent-child`, keyed `depends_on_id`) — so the doc's "read edges from the bulk dump only" rule is still right, but the membership test uses the other call. Unadapted, every id classifies OUT-OF-TREE: the Ready phase's structural fallback returns `ids: []` (round 1 exits via the empty-ready-set path reporting `completed: 0` — the silent no-op "Resolved in this branch" item 2 exists to prevent, back through another door) and `closeEpicsPrompt` can never close the root. Aggravating: the `--label sp:<epicId>` fast path is empty on a hand-imported tree, so the structural test is the only path.
- **Premise to verify:** bd's top-level `parent` is stable across supported versions.
- **Suggested fix shape:** read the top-level `parent` field, with the `dependency_type` lookup as a fallback for older bd; identity base case unchanged.

### 5. `planPrompt` is the only dispatch that names its working directory in prose, and the divergence guard's stated causes do not include the real one
- **Evidence:** `planPrompt` opens `Working directory: the integration worktree (see "Workspace and ledger" …)` — no path. Every other dispatch interpolates one (`In ${integrationWorktree}`, `cd ${im.branch}`, `Working directory: ${integrationWorktree}`). Run 1 round 1 died at the guard after 5 agents / 300,588 tokens / 1,055 s: the planner wrote a complete, correct 113 KB 24-task plan into the **session** worktree while the ledger sat in the **integration** worktree. The guard's own text names *"Two causes to check: planner-prompt.md's plan-file-name parameter was not honored … or the planner ran in a TASK worktree"* — neither was true. `scripts/sdd-workspace` resolves against `git rev-parse --show-toplevel` of the invoking cwd, so cwd does not affect the path — it **selects the workspace**. Reaches every run: super-code's own layout is `taskWorktree = ${integrationWorktree}/.worktrees/…`, nested by construction.
- **Premise to verify:** none.
- **Suggested fix shape:** interpolate `${integrationWorktree}` with a `cd` + `pwd` confirmation, and add "the dispatch never named a working directory" to the guard's causes.

### 6. Deferred minors are read only by the Finish reviewer, never by a later task's brief — a cross-task seam flagged at task 1 landed as a live P1 at task 16
- **Evidence:** `taskBriefPrompt` never consults the ledger; only the Finish reviewer is told minors are findings. Ledger line 15 (task 1, `9dv`): *"tread()'s NEG_INFINITY/INFINITY bounds … needing the design author's sign-off before a downstream setpoint/PI consumer relies on it"*. Task 16 (`iym`) then merged **as that consumer, unguarded**. Run 1's review reproduced it with a live probe: `t_star = ±inf` → `inf − inf = NaN` in the velocity-form PI → *"u is NaN forever with no recovery path"*, and because `NaN >= hi` is false the TARGET-UNREACHABLE rule never fires — the loop dies silently and hands NaN watts toward the actuator. Fix needed a whole new bead (`fw-fanctrl-loop-nez`), whose description reads *"No per-task review could see it: it is a cross-task seam … raised as a DEFERRED MINOR on 9dv."*
- **Premise to verify:** the coordinator can cheaply intersect an open minor's file against a later task's `filesTouched` — it holds both.
- **Suggested fix shape:** surface open minors whose file intersects a task's `filesTouched` into that task's brief — the seam review already does exactly this intersection for sibling commits, one round later and one merge too late.

### 7. The reviewer prompt classifies an unfailable assertion as "a Minor at least" and, in the same prompt, says minors never enter the fix loop — the defect is detected and routed to the one channel guaranteed not to fix it
- **Evidence:** `taskReviewPrompt`: *"an assertion the type makes unfailable … is a **Minor at least**"* and, four sentences on, *"minors never enter the fix loop"*. Ledger: **8 unfailable-assertion minors across 8 distinct beads**. Run 3's whole-epic review: *"in four of these the unfailable assertion is the only guard on the acceptance criterion it was written for … every reviewer caught it and every reviewer deferred it, so the loop never closed. Net effect: the 630-green is partly ornamental in precisely the spots the criteria called load-bearing."* Concretely, the `Freeze::Calibrating` wiring can be deleted and the suite stays green.
- **Premise to verify:** the reviewer can tell whether an assertion is the sole coverage of a brief-named acceptance criterion (it has the brief, the diff and the report).
- **Suggested fix shape:** carve out one non-deferrable case — "the only assertion covering an acceptance criterion named in the brief is unfailable" is NEEDS_FIX, not Minor.

### 8. `hotFileCap` is a count, not a lock: the plan can declare exclusive file ownership in prose and the scheduler has no primitive to honour it
- **Evidence:** the materialised plan states it: *"`src/control/controller.rs` … Tasks 4, 6, 9, 12, 13, 18, 19, 20 touch it … tasks 12, 19 and 20 own the file's body in turn"*, and Task 12's section: *"This task owns `src/control/controller.rs` **as a whole**"*. With `hotFileCap: 3` and `concurrency: 4`, three chains may hold the file concurrently, so the cap never bound: `Detector: round 1 — … hot-file deferrals: none`. Outcome, all three of run 1's failures on that one file: `Merge: mjv — rebase conflict: 1 files · gate fail → blocker` (~1700-line conflict), `Merge: zct — seam-review fixed · gate fail → blocker`, `Task 13 (dsh): BLOCKED — cannot proceed while its sibling 24s … is still open`. Run 1's review: *"a pipeline defect, not three task defects."* planner-prompt.md's *"worktree isolation makes dispatch-time collision impossible"* is true of dispatch and false of rebase, which is where the cost landed.
- **Premise to verify:** a per-file exclusivity flag is expressible in the mapping row the planner already returns (`{n, id, files}`).
- **Suggested fix shape:** let a mapping row mark a file exclusive (per-file cap of 1) and have `makeScheduler`'s `fileCounts` honour it.

### 9. The edge audit's arming condition compares a cumulative per-round dispatch count against the concurrency cap, so a working top-up hook makes it unarmable — 0 audits, including a round at peak in-flight 1 against cap 4
- **Evidence:** `frontierBelowCapStreak = dispatched.size < cap ? … + 1 : 0`, where `dispatched.size` is cumulative **including top-ups**. The three productive rounds: run 1 6 ready + 11 topped-up = 17; run 2 3 + 4 = 7; run 3 1 + 4 = 5 — all ≥ cap 4, so the streak never advanced and no `Edge audit:` line was ever written. Run 3 is exactly the thin-tail shape the audit exists to explain: `peak in-flight 1`, **6.7 h and 7.0M tokens for 5 merges**, versus run 1's 17 dispatches at width 4 in 2.7 h. The better the top-up works, the less likely the instrument that would explain the tail fires. (All three rounds have persisted `Detector:` lines — the data is complete.)
- **Premise to verify:** `sched.stats.peak` is a faithful concurrency measure (the detector already prints it).
- **Suggested fix shape:** arm on `sched.stats.peak < cap`, and emit the detector's below-cap hint off the same quantity.

### 10. The recurring-minor detector's clusters live only in coordinator memory and its signature is verbatim-only, so it never fired against 50 minors a human clustered into three classes, twice
- **Evidence:** `minorClusters` is a module-level `Map`, never persisted and never rebuilt on Resume, so the corpus resets at every invocation: **27 / 8 / 15 minors** per invocation, never the 50 the threshold would see. `minorSignature` normalises only case, quoting, hashes, paths, digits — reviewer-authored prose is one class in N phrasings. Result: **0 `Recurring minor:` lines** against clusters the run-3 reviewer found at 8/8, 5/5 and 4/4 distinct tasks. Both whole-epic reviews clustered by hand, and run 1's reached for a wrong mechanical explanation for the silence (see 11).
- **Premise to verify:** a Finish-time clustering pass over the ledger's `minor (deferred)` lines is affordable (the Finish reviewer already reads them all).
- **Suggested fix shape:** move clustering to a single Finish-time pass over the ledger — which also survives resumes for free — or say in the ledger that clustering is verbatim-only and a reviewer pass is still required.

### 11. `ledger-check` names one cause for a discrepancy it fired on twice, for two different causes, neither of them that one
- **Evidence:** the check's comment: *"the ledger-append path is lossy (a null dispatch drops a write silently)"*. It fired `M≠completed: 0 vs 15` in run 1 (a **reader** failure — defect 2) and `M≠completed: 20 vs 21` in run 2 (a **false `alreadyMerged` close** — defect 1). Both lines carried `append-failed 0` on the same output, which already rules the named cause out. The wrong pointer propagated: run 1's whole-epic review wrote *"The recurring-detector reads those counters, which is why it fired nothing"* — mechanically false (`noteRecurrence()` runs off in-memory clusters with no dependency on the Metrics parse).
- **Premise to verify:** none.
- **Suggested fix shape:** branch the message: `append-failed === 0` rules out loss; `mMerges === 0 && completed.size > 0` is a reader failure; `k` `complete` lines lacking a matching `Merge:` line names the false-close path.

### 12. `defaultTestPathspecs` is filename-shaped, so a language with inline unit tests gets a permanently-empty — and formally *valid* — `## Test changes` check on every review
- **Evidence:** the default list is `tests/**`, `spec/**`, `*_test.*`, `*.test.*`, `test_*.*` … and `**/` forms. This project had **no `tests/` directory at launch** and all 426 tests in `#[cfg(test)] mod tests` blocks inside `src/**/*.rs` (35 files). Every reviewing dispatch would report `Test changes: none` with a correctly-stated diff command — which `testChangesBlock`'s own rule declares **valid** — so the deleted/skipped/loosened-test check is silently dead on an epic whose plan deletes five modules and their tests. Second order: the correct list is not knowable until the plan exists (task 2 creates `tests/fixtures/…`), but `testPaths` must be supplied at launch; the `Launch:` lines show a relaunch purely to widen it.
- **Premise to verify:** pre-flight can check the default list against tracked files (`git ls-files -- <pathspecs>`).
- **Suggested fix shape:** apply the validity-floor principle — zero tracked matches at pre-flight is a loud warning, not a shipped no-op check; and say `testPaths` is effectively **required** for inline-test languages.

### 13. Six helper-script and prompt-template references are relative paths that resolve for a human reading the skill and for no dispatched agent
- **Evidence:** `scripts/task-brief`, `scripts/review-package`, `scripts/sdd-workspace`, `subagent-driven-development/{implementer,re-review,task-reviewer}-prompt.md` / `SKILL.md`, `./planner-prompt.md`, `./triage-prompt.md`. All live in the plugin cache; every agent's cwd is a worktree. The "Local adaptations" section has eight bullets and none covers this. The adaptation needed two module-level consts interpolated into all six.
- **Premise to verify:** the harness does not silently resolve skill-relative paths for a dispatched agent — if it does, this drops to a doc gap.
- **Suggested fix shape:** derive the two directories from `${CLAUDE_PLUGIN_ROOT}` (or accept them as contract keys); at minimum a "Local adaptations" bullet naming all six.

### 14. `sweepPrompt` hardcodes a test-runner report shape and validity floor, so a clean lint sweep and a catastrophically failed one render identically
- **Evidence:** the required line is `"<sha7> — <passed> passed, <failed> failed, …"` and the floor triggers on *"a passed count near zero"*. Three sweeps, same declared command (`cargo clippy --all-targets -- -D warnings`): run 3 `07b31dd — 0 passed, 0 failed, 0 errors, 0 skipped; failing: none` **plus an agent-appended disclaimer**; run 1 `9f112ed — 0 passed, 0 failed, 83 errors`; run 2 `MEASUREMENT INVALID`. One field, three meanings, and the clean case is byte-shaped like the floor's own failure trigger. Related: `config.sweep` is a single command, so declaring a lint sweep forfeits the branch-wide *test* measurement — run 3's reviewer had to run `cargo test` itself.
- **Premise to verify:** none.
- **Suggested fix shape:** `<tip> — <PASS|FAIL|INVALID> — <free-form detail>`; and/or let `sweep` be a list of declared commands, each with its kind.

### 15. The sweep runs only at Finish, so there is no baseline and a mid-epic regression in the swept dimension is indistinguishable from pre-existing breakage
- **Evidence:** run 3's review, cluster C3: clippy red for most of the epic across four beads, and *"task 22 shipped **7 genuinely new lints** … under cover of that pre-existing breakage, with the report calling plain `cargo clippy` 'clean' … a gate which is knowingly red is not a gate."*
- **Premise to verify:** the sweep command is runnable at pre-flight against the integration branch's starting tip.
- **Suggested fix shape:** run the declared sweep once at pre-flight as `Sweep: baseline <tip> — …` and let the Finish sweep report a delta.

### 16. `authRefusalRule()` cannot distinguish "you may not do this" from "say that again more simply", and says nothing about shell cwd persistence in a layout the skeleton itself makes nested
- **Evidence:** `BLOCKED_AUTH` is defined as *"the tool call itself is declined — the command never executed: no exit code, no output"*. A harness refusal citing command **form** matches that word for word — reproduced independently in the analyst's own session (an `awk` call refused as "too complex to verify"; the identical work split across two calls ran immediately), and twice in the run's pre-flight (a heredoc whose *payload* contained the word `git`; a `bd create` whose title came through `$(…)`). An agent following the rule verbatim quarantines the task for the run. Separately: super-code nests task worktrees inside the integration worktree by construction; one `cd <shared checkout> && bd …` left the persistent shell there, after which the isolation guard refused **every** subsequent call including bare `cd` and `pwd` (it evaluates the *starting* cwd), recoverable only via a non-shell tool.
- **Premise to verify:** other harnesses have an analogous static-form classifier and persistent cwd; if Claude Code's are unique this narrows to a harness-specific note.
- **Suggested fix shape:** name a third category in the rule (a refusal citing *form* is re-spell-and-retry, never `BLOCKED_AUTH`), and add a shell-discipline clause: absolute paths, absolute `cd` first in every call, never rely on a prior call's cwd — `authRefusalRule()` is already the one string every git/bd-running dispatch interpolates.

## Run metrics

### Judge panel
none — no `super-roast` ran in this execution phase.

### Fix loop
- run 1: `Metrics: fix-loop round 1: 0 addressed / 0 entered · … · round 5: 0 addressed / 0 entered`, `breaker-tripped: 0` — **all zero by defect 2, not a measurement**
- run 2: `round 1: 2 addressed / 2 entered`, rounds 2–5 zero, `breaker-tripped: 0`
- run 3: `round 1: 6 addressed / 6 entered`, rounds 2–5 zero, `breaker-tripped: 0`
- raw `fix round` lines on the ledger: 7 (6 distinct ids; the parse keeps only from each id's last `round 1`, by design)

### Merge-back
- final: `Metrics: merges 25 · merge-failed 2 · rebase-conflicts 1 · seam-reviews 9 (fixed 1) · gate-fails 2`; `Metrics: ledger-check ok · append-failed 0 · append-retried 1`
- raw `Merge:` lines: 27 (16 / 6 / 5 per invocation); seam review fired 9 (`cleared` 8, `fixed` 1), `none` 18; rebase conflict 1 (`mjv`); `gate fail → blocker` 2 (`zct`, `mjv`)
- deferred minors 50 across 21 beads; `Recurring minor:` / `Recurring blocker:` 0; `Edge audit:` 0; `parked` / `BLOCKED-AUTH` 0
- `Detector:` lines 3 (one per invocation, all `round 1` — every productive round is measured)

### Coverage
none — no `run.md`, no coverage-round `requirements:` or fix-round `scope-filter:` lines.

### Bead graph
35 ids under the `fw-fanctrl-loop` prefix: 25 task beads parented to the root plus the root; 6 blocker/bug beads unparented; 2 malformed blocker beads parented to the root (`6ma.1`, `6ma.2`). Tree is flat — every task a direct child of the root — and carries no `sp:` labels (hand-imported). Edge reasons: the beads carry `blocked-by fwloop.N:` lines keyed by the *design ordinal*, not the bead id, so the extractor below renders them `unstated`; the reasons exist in each description.

| id | type | title | what (first sentence of description) |
| --- | --- | --- | --- |
| `fw-fanctrl-loop-6ma` | epic | Close the loop on fw-fanctrl (no learned thermal model) | Root epic. |
| `fw-fanctrl-loop-0nv` | task | Calibration step test | src/calib/runner.rs: |
| `fw-fanctrl-loop-24s` | task | Remove the adaptation tier | src/control/controller.rs: |
| `fw-fanctrl-loop-2ll` | task | BLOCKED: fw-fanctrl-loop-9it — ryzenadj/ryzen_smu cannot i | Task fw-fanctrl-loop-9it ("Spike: |
| `fw-fanctrl-loop-438` | task | Controller hooks: warm-start, refinement, calibration | src/control/controller.rs: |
| `fw-fanctrl-loop-4aj` | task | FOPDT fit + IMC gain derivation | src/calib/fopdt.rs: |
| `fw-fanctrl-loop-51b` | task | fw-fanctrl emulator + chained plant | src/test_support/plant.rs (cfg(test)): |
| `fw-fanctrl-loop-52c` | task | EC replica + NVMe + AC sensors | src/sensors/ec.rs: |
| `fw-fanctrl-loop-58u` | task | fw-fanctrl socket client | src/fanctrl/client.rs: |
| `fw-fanctrl-loop-6ma.1` | task | Gate red on fw-fanctrl-loop-zct merge: adaptation-tier tes | cargo test is red on the integration merge for fw-fanctrl-loop-zct, per the declared per-merge gate. |
| `fw-fanctrl-loop-6ma.2` | task | Rebase blocker: task-fw-fanctrl-loop-mjv vs epic controlle | Rebasing task branch task-fw-fanctrl-loop-mjv (fw-fanctrl-loop-mjv, tip 599cbf856ae353b2209d110d2696b91d0bea0feb) onto epic-fw-fanctrl-loop-6ma-integr… |
| `fw-fanctrl-loop-7e9` | task | BLOCKED: fw-fanctrl-loop-j6s — §2.4 anti-windup rule not r | Task fw-fanctrl-loop-j6s ("Controller loop integration") is blocked by fw-fanctrl-loop-9it ("Spike: |
| `fw-fanctrl-loop-7ij` | task | README + docs | README: |
| `fw-fanctrl-loop-834` | task | Budget integrator | src/control/budget.rs: |
| `fw-fanctrl-loop-9dv` | task | Curve model + DutyRpmTable | New src/fanctrl/{mod,curve,table}.rs. |
| `fw-fanctrl-loop-9it` | task | Spike: settle the anti-windup rule | **A spike, not production code** — its output is a decision written into §2.4, and its harness is either deleted or kept as a #[cfg(test)] fixture, wh… |
| `fw-fanctrl-loop-a5j` | bug | mirror_decision never syncs TargetUnreachable into Control | Controller::mirror_decision (src/control/controller.rs) syncs exactly five StatusFlag variants from Decision.flags into self.status.flags: |
| `fw-fanctrl-loop-a78` | bug | GPU_TRIP_C (87C) sits below GPU_HOT_C_DEFAULT (90C): the h | src/control/watchdog.rs: |
| `fw-fanctrl-loop-blm` | task | Capture machine fixtures + test-support layout | tests/fixtures/: |
| `fw-fanctrl-loop-bwt` | task | fw-fanctrl-loop-dsh blocked: PersistedState schema change  | Task fw-fanctrl-loop-dsh (Persisted state migration, sdd plan task 13) cannot be completed without either leaving cargo test red or absorbing sibling … |
| `fw-fanctrl-loop-cm7` | task | Closed-loop acceptance + configuration smoke | Controller-level sims on ChainedPlant through the real on_sample, with fwloop.19's hooks active, the load step being a utilisation + watts step: |
| `fw-fanctrl-loop-dsh` | task | Persisted state migration | src/state.rs: |
| `fw-fanctrl-loop-eyi` | task | Deletion sweep | Delete src/control/thermal_model.rs, kalman.rs, trust.rs, cooldown.rs, trim.rs, their mod.rs lines, and every remaining reference (the controller's ti… |
| `fw-fanctrl-loop-fo1` | task | Controller status surface | src/control/controller.rs types only: |
| `fw-fanctrl-loop-hwg` | bug | Controller::on_sample's resume handler never clears AutoSt | src/control/controller.rs's on_sample resume branch (the if s.resumed { ... |
| `fw-fanctrl-loop-iym` | task | Mode arbiter, reconciliation, feasibility | src/control/mode.rs: |
| `fw-fanctrl-loop-j6s` | task | Controller loop integration | src/control/controller.rs, on the tier-free controller from fwloop.22: |
| `fw-fanctrl-loop-jpg` | task | Actuator read-back | src/actuators/cpu.rs: |
| `fw-fanctrl-loop-mjv` | task | TUI + telemetry surface | src/ui/view.rs: |
| `fw-fanctrl-loop-mm2` | task | Guards (dGPU, NVMe) + config keys | src/control/guards.rs: |
| `fw-fanctrl-loop-nez` | task | Guard the infinite tread endpoint: Curve::t_star can be +/ | Confirmed defect in MERGED code on epic-fw-fanctrl-loop-6ma-integration @ 9f112ed. |
| `fw-fanctrl-loop-nsc` | task | Integration sweep: fw-fanctrl closed loop | Verify the goal's main flows end to end on the merged tree and implement what is missing: |
| `fw-fanctrl-loop-sov` | task | Sample plumbing + socket poller | src/types.rs Sample gains ec: |
| `fw-fanctrl-loop-wwh` | task | super-code auth probe | Pre-flight standing-authorisation probe. |
| `fw-fanctrl-loop-zct` | task | Allocator: scalar budget split | src/control/allocator.rs: |

| dependent | blocker | reason (from `blocked-by` line, or `unstated`) |
| --- | --- | --- |
| `fw-fanctrl-loop-0nv` | `fw-fanctrl-loop-dsh` | unstated |
| `fw-fanctrl-loop-0nv` | `fw-fanctrl-loop-4aj` | unstated |
| `fw-fanctrl-loop-0nv` | `fw-fanctrl-loop-834` | unstated |
| `fw-fanctrl-loop-24s` | `fw-fanctrl-loop-fo1` | unstated |
| `fw-fanctrl-loop-438` | `fw-fanctrl-loop-0nv` | unstated |
| `fw-fanctrl-loop-438` | `fw-fanctrl-loop-j6s` | unstated |
| `fw-fanctrl-loop-4aj` | `fw-fanctrl-loop-834` | unstated |
| `fw-fanctrl-loop-51b` | `fw-fanctrl-loop-blm` | unstated |
| `fw-fanctrl-loop-51b` | `fw-fanctrl-loop-58u` | unstated |
| `fw-fanctrl-loop-51b` | `fw-fanctrl-loop-sov` | unstated |
| `fw-fanctrl-loop-51b` | `fw-fanctrl-loop-9dv` | unstated |
| `fw-fanctrl-loop-51b` | `fw-fanctrl-loop-52c` | unstated |
| `fw-fanctrl-loop-52c` | `fw-fanctrl-loop-blm` | unstated |
| `fw-fanctrl-loop-58u` | `fw-fanctrl-loop-blm` | unstated |
| `fw-fanctrl-loop-6ma.1` | `fw-fanctrl-loop-24s` | unstated |
| `fw-fanctrl-loop-6ma.1` | `fw-fanctrl-loop-eyi` | unstated |
| `fw-fanctrl-loop-7ij` | `fw-fanctrl-loop-58u` | unstated |
| `fw-fanctrl-loop-7ij` | `fw-fanctrl-loop-mm2` | unstated |
| `fw-fanctrl-loop-7ij` | `fw-fanctrl-loop-fo1` | unstated |
| `fw-fanctrl-loop-7ij` | `fw-fanctrl-loop-0nv` | unstated |
| `fw-fanctrl-loop-9it` | `fw-fanctrl-loop-834` | unstated |
| `fw-fanctrl-loop-cm7` | `fw-fanctrl-loop-j6s` | unstated |
| `fw-fanctrl-loop-cm7` | `fw-fanctrl-loop-51b` | unstated |
| `fw-fanctrl-loop-cm7` | `fw-fanctrl-loop-438` | unstated |
| `fw-fanctrl-loop-dsh` | `fw-fanctrl-loop-9dv` | unstated |
| `fw-fanctrl-loop-dsh` | `fw-fanctrl-loop-834` | unstated |
| `fw-fanctrl-loop-dsh` | `fw-fanctrl-loop-blm` | unstated |
| `fw-fanctrl-loop-eyi` | `fw-fanctrl-loop-zct` | unstated |
| `fw-fanctrl-loop-eyi` | `fw-fanctrl-loop-24s` | unstated |
| `fw-fanctrl-loop-eyi` | `fw-fanctrl-loop-0nv` | unstated |
| `fw-fanctrl-loop-eyi` | `fw-fanctrl-loop-mjv` | unstated |
| `fw-fanctrl-loop-iym` | `fw-fanctrl-loop-fo1` | unstated |
| `fw-fanctrl-loop-iym` | `fw-fanctrl-loop-58u` | unstated |
| `fw-fanctrl-loop-iym` | `fw-fanctrl-loop-52c` | unstated |
| `fw-fanctrl-loop-iym` | `fw-fanctrl-loop-9dv` | unstated |
| `fw-fanctrl-loop-j6s` | `fw-fanctrl-loop-jpg` | unstated |
| `fw-fanctrl-loop-j6s` | `fw-fanctrl-loop-iym` | unstated |
| `fw-fanctrl-loop-j6s` | `fw-fanctrl-loop-834` | unstated |
| `fw-fanctrl-loop-j6s` | `fw-fanctrl-loop-9it` | unstated |
| `fw-fanctrl-loop-j6s` | `fw-fanctrl-loop-52c` | unstated |
| `fw-fanctrl-loop-j6s` | `fw-fanctrl-loop-9dv` | unstated |
| `fw-fanctrl-loop-j6s` | `fw-fanctrl-loop-sov` | unstated |
| `fw-fanctrl-loop-j6s` | `fw-fanctrl-loop-mm2` | unstated |
| `fw-fanctrl-loop-j6s` | `fw-fanctrl-loop-fo1` | unstated |
| `fw-fanctrl-loop-j6s` | `fw-fanctrl-loop-24s` | unstated |
| `fw-fanctrl-loop-j6s` | `fw-fanctrl-loop-dsh` | unstated |
| `fw-fanctrl-loop-j6s` | `fw-fanctrl-loop-zct` | unstated |
| `fw-fanctrl-loop-jpg` | `fw-fanctrl-loop-blm` | unstated |
| `fw-fanctrl-loop-mjv` | `fw-fanctrl-loop-sov` | unstated |
| `fw-fanctrl-loop-mjv` | `fw-fanctrl-loop-fo1` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-52c` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-58u` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-834` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-7ij` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-9it` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-0nv` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-blm` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-24s` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-51b` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-mjv` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-438` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-9dv` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-jpg` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-cm7` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-4aj` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-iym` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-nez` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-fo1` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-zct` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-dsh` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-eyi` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-j6s` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-sov` | unstated |
| `fw-fanctrl-loop-nsc` | `fw-fanctrl-loop-mm2` | unstated |
| `fw-fanctrl-loop-sov` | `fw-fanctrl-loop-52c` | unstated |
| `fw-fanctrl-loop-sov` | `fw-fanctrl-loop-58u` | unstated |

## Design questions

### Should the Finish reviewer's MUST-FIX items be auto-filed as beads?
- **Evidence:** run 1's Finish review produced three MUST-FIX items and every one needed a human between invocations: `9it` reopened by hand; the ±∞ seam filed by hand as `fw-fanctrl-loop-nez` (~40 s after the review was written); the `24s → dsh` ordering edge never encoded at all.
- **For:** the run already knows how to file an unparented, `blocker`-labelled bead from four sites; auto-filing would let a `ready-drained` invocation be relaunched unattended and actually converge, which is what three identical invocations cost here.
- **Against:** the whole-epic review is a judgment artifact; auto-filing turns reviewer prose into tracker mutations no one adjudicated — precisely the class of unverified mutation defect 1 shows going wrong.
- If upstream decides otherwise, please state the position explicitly so downstream can reconcile against words rather than silence.

### Should the planner be required to materialise ordering it states in prose as `bd` blocking edges?
- **Evidence:** planner-prompt.md tells the planner to assign ordinals "in dependency order" and to reason about hot files, but never to write an edge; the ready set comes purely from `bd`. Run 1's review, "Recommended order to unblock" item 3: *"Add the `blocks` edge `24s → dsh` … the plan already states this ordering; beads never encoded it."*
- **For:** it is the only channel the scheduler reads, and defect 8's cost was three escalations.
- **Against:** super-design's §Decomposition deliberately discourages edges that encode narrative order, and the edge audit exists to *name* such edges as suspect. Defect 8's per-file exclusivity flag may be the cheaper answer to the same problem.
- If upstream decides otherwise, please state the position explicitly so downstream can reconcile against words rather than silence.

## Doc gaps

- **The id-prefix "sanity check" in `closeEpicsProcedure` step 2b disagrees with the authoritative test on 100% of ids in any tree whose ids are hash-suffixed rather than dotted.** This epic's children are `fw-fanctrl-loop-<3-char hash>` under root `fw-fanctrl-loop-6ma`: they share leading text with the root but satisfy neither `=== epicId` nor `startsWith(epicId + '.')`. The dotted convention holds only for beads the run itself creates via `bd create --parent` (`6ma.1`, `6ma.2`) — never for an imported tree. Either drop the cross-check or gate it on "the epic's children actually use the dotted convention", and say so.
- **"Local adaptations" does not mention that the helper scripts and prompt templates are outside the repo** (defect 13) nor that `testPaths` is effectively required for inline-test languages (defect 12).
- **The `alreadyMerged` Known Limitation reads as theoretical** ("a live run is the only check"). The live run happened; the outcome above belongs beside it.

## Already fixed — do not re-litigate

- **Issue #5 defect 9's elided-retry for classifier-refused ledger appends worked as designed.** `ledger-append:merge:fw-fanctrl-loop-9it` was refused by the safety classifier in runs 1 and 3 (surfaced as the run's single `agents_error` with a SECURITY WARNING banner); `appendLedger()`'s one retry with the free text elided landed the line both times — `Metrics: ledger-check ok · append-failed 0 · append-retried 1`. Recording as a success. The residual — the refusal still surfaces as a hard error that nothing connects to the successful retry — is harness surfacing, not skill-actionable.
- **Issue #5 defects 1–2 (pinned `taskWorktree`/`taskBranch`)** held: no duplicate worktrees, no agent-chosen branch names, across 25 tasks and 3 invocations.
- **The divergence guard (defect 5's catcher)** did its job — the plan/ledger split was caught at 5 agents, not discovered 40 reviews later.

## Not established

- **Parallelism detector as `log()` output** was not captured by the invoking session; the ledger's three persisted `Detector:` lines are the only record. They cover every productive round (each invocation ran exactly one before draining), so nothing is missing — but per-top-up timing behind run 3's `peak in-flight 1` is not reconstructible.
- **Run 2's single escalation is inferred** from `fw-fanctrl-loop-7e9`'s title and creation timestamp inside run 2's window; no run-2 Finish review was saved.
- **No roast ran**, so Judge panel and Coverage are genuinely `none`, not omitted.
- **Defect 9's speed claim is bounded:** run 3's `peak in-flight 1` may reflect a genuine dependency chain. The finding is that the instrument built to answer that question could not arm — not that the chain was avoidable.
- **Defect 8 is a single-epic observation** with one unusually hot file (8 of 25 tasks declared `controller.rs`). The mechanism is general; the default's suitability across projects is not.
- By default: the coordinator's ledger-append path is fire-and-forget and can lose a line without the coordinator noticing, so every ledger-derived count in `## Run metrics` is a lower bound. `Metrics: ledger-check ok · append-failed 0 · append-retried 1` is the one cross-check, and it reads clean on the final invocation.
- **The analyst that produced these findings ran on opus** (fresh context, dispatched after the run); the invoking session triaged and confirmed four of its corrections to that session's own friction log.

## Verification bar

Before trusting fixes for the above, upstream should: (a) run one **live** (non-dryRun) invocation and assert every Finish `Metrics:` counter is non-zero against a ledger with merges — the dryRun harness structurally cannot catch defect 2; (b) grep the skeleton for dispatches that read a structured field off a schema-less result; (c) run a live blocker-filing through the **merge gate** specifically and check the created bead's `parent`, `labels` and `dependencies` (defect 3); (d) dispatch `planPrompt` from a cwd that is not the integration worktree in a nested-worktree repo (defect 5); (e) run the whole skeleton against a `bd` ≥ 1.2.x tree with hash-suffixed ids and no `sp:` labels (defect 4, doc gap 1); (f) run one round with a top-up-heavy graph and assert the edge audit still arms when `peak < cap` (defect 9); (g) re-dispatch a task whose branch sits at an integration commit with zero own commits and assert `alreadyMerged: false` reaches the coordinator (defect 1).

---
If a premise above is wrong, stop and say so rather than improvising a larger change.

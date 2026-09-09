# Friction log — super-code run, epic fw-fanctrl-loop-6ma

Run: `wf_8e35e7de-079`. Skeleton: superpowers-alepar/superpowers 6.3.0-alepar3.8.
To be merged into `<workspace>/friction.md` in the integration worktree once the planner
creates it, and analysed by `superpowers:upstream-feedback` at Finish (this session owns
the finish, so per coordinator-workflow.md §Finish the analysis runs BEFORE the worktree
is deleted).

## Pre-flight

### 1. `treeMembershipTest` is written against a `bd` version that no longer reports `dependency_type`
- **What happened:** `coordinator-workflow.md`'s shared `treeMembershipTest` instructs the
  agent to find, in `bd show <id> --json`'s `dependencies` array, an entry with
  `dependency_type: "parent-child"`. On bd 1.2.2 those entries carry only
  `id`/`title`/`description` — no `dependency_type` at all — and `dependencies` lists
  *blocking* deps. The parent link is a **top-level `parent` field** instead.
- **Consequence if unadapted:** every id classifies OUT-OF-TREE. Both call sites break at
  once: the Ready phase's structural fallback returns `ids: []` (round 1 exits via the
  empty-ready-set quarantine path reporting `completed: 0` — the exact silent no-op the
  doc's "Resolved in this branch" item 2 was written to prevent, reintroduced through a
  different door), and `closeEpicsPrompt` can never close the root, so `rootClosed` never
  goes true.
- **Aggravating factor:** the fast path (`--label sp:<epicId>`) is empty on this epic (no
  `sp:` labels — the tree was hand-imported), so the structural fallback is not a fallback
  here, it is the only path.
- **Repro:** `bd show <any child> --json | python3 -c "import json,sys; print(json.load(sys.stdin)[0]['dependencies'][0].keys())"` on bd 1.2.2.
- **Evidence:** bd version 1.2.2 (6c124203e). `bd show fw-fanctrl-loop-9it --json` →
  top-level `parent: "fw-fanctrl-loop-6ma"`; root reports `parent: null`.
  The doc's own "Local adaptations" bullet already warns that `bd show --json`
  underreports edges as of bd 1.0.5 — but it frames that as a *completeness* problem for
  graph reasoning, not as a *schema* change that breaks the membership test the doc
  mandates two phases use.
- **Adaptation applied:** rewrote `treeMembershipTest` to walk the top-level `parent`
  field, identity base case unchanged.

### 2. The id-prefix "sanity check" is actively wrong for a hash-suffixed bead tree
- **What happened:** `closeEpicsProcedure` step 2b tells the agent the id-prefix convention
  (`<id> === epicId` or `<id>` starts with `epicId.`) "should agree with" the structural
  test. This epic's ids are `fw-fanctrl-loop-<3-char hash>` under root
  `fw-fanctrl-loop-6ma` — they share a text prefix with the root but satisfy neither
  clause, so the check disagrees with the structural test on **every** in-tree bead.
- **What the user said:** User didn't comment; observed by the model.
- **Consequence:** a disagreement on 100% of ids is noise that invites an agent to
  second-guess the authoritative test. The doc says "trust (a)", so it is not a
  correctness bug — but a cross-check that always fires is worse than no cross-check.
- **Adaptation applied:** step 2b replaced with an explicit "no prefix check in this run,
  and here is why" note.

### 3. Helper scripts and prompt templates are referenced by relative path, but live outside the repo
- **What happened:** every dispatch names `scripts/task-brief`, `scripts/review-package`,
  `scripts/sdd-workspace`, `subagent-driven-development/implementer-prompt.md`,
  `./planner-prompt.md`, `./triage-prompt.md`. All six live in the plugin cache
  (`~/.claude/plugins/cache/...`), not in the project, and every agent's cwd is a task
  worktree — so each is unresolvable as written.
- **Consequence if unadapted:** the brief stage fails on task 1 and every task after it.
- **Note:** this is arguably the single most portability-relevant gap in the skeleton, and
  it is not called out under "Local adaptations". The `./`-relative forms in particular
  read as "relative to the skill directory", which is true for a human reading the doc and
  false for every dispatched agent.
- **Adaptation applied:** two module-level consts (`SDD_DIR`, `SC_DIR`) interpolated into
  all six references.

### 4. The default `testPaths` list matches nothing in a Rust repo with inline tests
- **What happened:** `defaultTestPathspecs` covers `tests/**`, `spec/**`, and
  `*_test.*`/`*.test.*`/`test_*.*` filename patterns. This project has **no** `tests/`
  directory; all 426 tests are `#[cfg(test)] mod tests` blocks inside `src/**/*.rs`
  (35 files).
- **Consequence if unadapted:** every reviewing dispatch's `## Test changes` block reports
  "none" with a correctly-stated diff command — i.e. *valid* by the doc's own rule, and
  therefore silently dead. The check that exists to catch a deleted/skipped/loosened test
  would never fire, on an epic whose design deletes five modules and their tests.
- **Adaptation applied:** `config.testPaths: ["src/**/*.rs"]`.
- **Upstream suggestion:** the default list is language-shaped (Python/JS/Go). Inline unit
  tests (Rust, C++, some Java) have no filename signal at all, so *any* default will miss
  them — worth stating in the contract that `testPaths` is effectively **required** for
  such projects, rather than optional-with-a-safe-default.

### 5. Shell working directory persists between tool calls and poisoned this session mid-pre-flight
- **What happened:** one `cd <shared checkout> && bd create ...` (run there deliberately,
  to reach the shared `.beads` DB) left the persistent shell cwd in the shared checkout.
  This session is worktree-isolated, so the isolation guard then refused **every**
  subsequent Bash call — including a bare `cd <correct worktree>` and a bare `pwd` —
  because it evaluates the *starting* cwd before running the command. No shell command
  could restore the shell. Recovered only via the `EnterWorktree` tool.
- **What the user said:** "The Bash tool's working directory persists between calls. A cd
  into a sub-worktree silently sent later relative-path edits into the wrong checkout
  during the design session. Prefer absolute paths in every dispatched agent's commands."
  — the user pre-warned about exactly this class; the failure mode hit was the harsher
  variant (unrecoverable-by-shell, not merely silently-wrong).
- **Evidence:** three consecutive refusals of `cd …`, `pwd`, and `git -C … ls-files`, all
  with "this command's working directory resolved to the shared checkout".
- **Adaptation applied:** a SHELL DISCIPLINE paragraph appended to `authRefusalRule()` (the
  one string already interpolated into every git/bd-running dispatch), telling agents to
  make an absolute `cd` the first statement of every call and never to rely on a prior
  call's cwd.

### 6. The isolation guard refuses compound commands as "too complex to verify" — and that is not a permission refusal
- **What happened:** two calls were refused not because the operation was disallowed but
  because the command was "too complex to verify" — one a heredoc whose *payload text*
  contained the word `git`, one a `bd create` whose title was interpolated through `$(...)`.
  Both operations were separately confirmed allowed.
- **Consequence:** an agent following `authRefusalRule()` verbatim would classify this as
  `BLOCKED_AUTH`, quarantine the task for the whole run, and record lost coverage — for a
  refusal that a plain re-spelling of the same command resolves immediately. The doc's
  `BLOCKED_AUTH` definition ("the tool call itself is declined — the command never
  executed: no exit code, no output") matches this case exactly, which is the problem: the
  definition cannot distinguish "you may not do this" from "say that again more simply".
- **Adaptation applied:** the SHELL DISCIPLINE paragraph explicitly rules this out of
  `BLOCKED_AUTH` and instructs a split-and-retry.
- **Upstream suggestion:** `authRefusalRule()` should name this third category directly.

## Run

### 7. `planPrompt` is the only dispatch that names its working directory in prose — and the planner obeyed its own cwd instead (BLOCKING, cost one round)
- **What happened:** run `wf_8e35e7de-079` died at the Plan-phase divergence guard on round 1.
  The planner wrote a complete, correct, 113 KB, 24-task plan to
  `<SESSION worktree>/.superpowers/sdd/fw-fanctrl-loop-6ma-plan/` while the ledger sat in
  `<INTEGRATION worktree>/.superpowers/sdd/fw-fanctrl-loop-6ma-plan/`.
- **Cause (verified, not inferred):** `planPrompt` opens with `Working directory: the
  integration worktree (see "Workspace and ledger" ...)` — a PROSE reference, with no path.
  Every other dispatch in the skeleton interpolates the real path: `In ${integrationWorktree}`
  (readLedgerPrompt, ledgerAppendPrompt, sweepPrompt, mergePrompt, closeOnlyPrompt),
  `cd ${im.branch}` (taskReviewPrompt, reReviewPrompt, seamReviewPrompt), `Working directory:
  ${integrationWorktree}` (edgeAuditPrompt). The planner therefore had nothing to obey and used
  the cwd it was spawned in. `scripts/sdd-workspace` resolves its output against
  `git rev-parse --show-toplevel` of the invoking cwd, so in a repo checked out as nested
  worktrees the cwd does not merely affect the path — it *selects the workspace*.
- **Why this is worth reporting even though nothing was lost:** the guard is excellent and did
  exactly what it promises. But read its own error text — "Two causes to check: planner-prompt.md's
  plan-file-name parameter was not honored by this dispatch, or the planner ran in a TASK worktree
  instead of the integration worktree." Neither was true. The real cause — *the dispatch never
  told the planner where to stand* — is not among the causes the guard suggests, and the guard's
  surrounding comment block reasons at length about a MISBEHAVING planner (a stale cached
  template, a manual invocation) without ever noting that a well-behaved planner has no path to
  honour in the first place. The comment even says a correct planner "legitimately reports a path
  prefixed by the integration worktree" — which is only true if something put it there.
- **Consequence for anyone who removes the guard:** the guard is load-bearing in a way its own
  prose understates. Without it this run would have silently split plan-from-ledger and every
  downstream `task-brief`/`review-package`/report path would have resolved into a workspace no
  other stage reads — the exact "~40 reviews of an empty package" class the skeleton fixed
  elsewhere, reintroduced one directory up.
- **Repro:** dispatch `planPrompt` from any cwd that is not the integration worktree, in a repo
  with nested worktrees.
- **Evidence:** run `wf_8e35e7de-079`; 5 agents, 0 errors, 300 588 subagent tokens, 1 055 s
  before the throw. Guard message quoted above. Skeleton
  6.3.0-alepar3.8, `coordinator-workflow.md` `planPrompt`.
- **Fix applied:** `planPrompt` now interpolates `${integrationWorktree}`, requires `cd` +
  `pwd` confirmation as its first action, and explains why cwd selects the workspace. Also added
  an explicit reuse-don't-regenerate clause for the resume case (planner-prompt.md rule 3),
  since the resumed planner re-enters with all 24 mapping rows already written and a
  regeneration would renumber ordinals the ledger and brief filenames are keyed to.
- **Suggested upstream fix:** one line — interpolate the path — plus adding "the dispatch did not
  name a working directory" to the guard's list of causes to check.

### 9. `read-ledger:finish` is missing `schema: LEDGER_TEXT` — the entire Metrics block silently reads zero (BLOCKING for the feature; invisible to dryRun by construction)
- **What happened:** run 1 emitted `Metrics: merges 0 · merge-failed 0 · rebase-conflicts 0 ·
  seam-reviews 0 (fixed 0) · gate-fails 0`, all fix-loop rounds `0 addressed / 0 entered`,
  `breaker-tripped: 0`, and `ledger-check M≠completed: 0 vs 15` — against a ledger whose body
  carried **16 `Merge:` lines, 2 rebase conflicts, 3 `gate fail → blocker`, 3 seam reviews and 2
  fix rounds**.
- **Cause (verified in the skeleton, not inferred):** the Resume-phase dispatch is
  `{ label: 'read-ledger', phase: 'Resume', schema: LEDGER_TEXT, model: ... }`; the Finish-phase
  one is `{ label: 'read-ledger:finish', phase: 'Finish', model: ... }` — **no `schema`**. Without
  a schema the Workflow runtime returns the agent's final text as a plain **string**, so
  `metricsLedger?.text` is `undefined`, `(undefined || '')` parses to zero lines, and every
  derived count is 0. Confirmed against the journal: only ONE result in the whole run has a
  `text` key (the Resume read), and `read-ledger:finish` returned a bare string.
- **Why no test catches it:** `pick()` returns the declared stub object directly under
  `dryRun: true` and never exercises the schema mechanism at all. A `read-ledger:finish` stub of
  the form `{text: '...'}` makes every dryRun assertion pass on a script that cannot work live.
  **No assertion over stubbed dispatches can ever catch a missing-schema defect** — this is a
  structural blind spot in the dryRun harness, not a gap in the scenarios.
- **Second-order harm:** `ledger-check` reports the discrepancy as a suspected *ledger* loss
  ("the ledger-append path is lossy — a null dispatch drops a write silently"), which points a
  reader at the wrong subsystem entirely. Nothing was lost; the reader never received the text.
  Suggest the check distinguish "M is 0 while completed is non-zero" as a reader failure.
- **Third-order harm:** run 2's whole-epic reviewer read the zeroed counters and concluded the
  recurring-minor detector was broken because it "reads those counters". It does not —
  `noteRecurrence()` runs off in-memory clusters at the merge gate and in `handleBlocker`, with no
  dependency on the Metrics parse. So a cosmetic-looking defect propagated into a false mechanical
  claim in the run's own final review.
- **Fix applied:** added `schema: LEDGER_TEXT` to the Finish-phase dispatch.

### 10. Known Limitation 3 fired live: a false `alreadyMerged` closed the epic's single most important bead with zero work done
- **What happened:** `fw-fanctrl-loop-9it`, the §2.4 anti-windup SPIKE — the bead the operator
  singled out in the invocation as the one an implementer must never improvise around — was
  closed with the ledger line `complete (already merged into
  epic-fw-fanctrl-loop-6ma-integration before this re-entry — bead closed, no new review)`.
  **Nothing was merged and nothing was written.**
- **Sequence:** run 1's implementer hit the `ryzen_smu` hardware precondition and reverted its own
  changes → triage returned RESOLVE with a correct, detailed procedure → the same-round RESOLVE
  retry re-entered at the brief stage → the brief agent answered `alreadyMerged: true` → the
  coordinator short-circuited to `closeOnlyPrompt` and ran `bd close`.
- **Why the answer was wrong:** `task-fw-fanctrl-loop-9it` sat at exactly an integration-branch
  commit with **zero commits of its own**. `taskBriefPrompt` states the correct rule explicitly —
  *"a tip equal to the integration tip, or appearing only as a FIRST parent, is NOT merged"* — and
  the agent violated the instruction it was given. The coordinator has no git access and so
  accepts the answer on trust; the doc names exactly this ("a wrong `true` closes a bead whose
  work never merged... a live run is the only check of the latter").
- **Consequence, and why it is the worst possible bead to lose:** a *closed* bead is invisible to
  `bd ready` forever, so the epic's root blocker vanished from the queue while the blockage
  remained. Run 2 then drained with 21/24 complete and the headline deliverable absent. Only the
  whole-epic reviewer's independent check of the four acceptance criteria caught it. **A silent
  false-close is strictly worse than a blocker bead**: the escalation currency exists precisely so
  that nothing disappears, and this path bypasses it.
- **Verified absent at tip 247c2d1:** `src/control/spike_antiwindup.rs` does not exist; design doc
  line 344 still reads "Open for the spike to decide and record: the predicate itself; ...
  `DEMAND_MARGIN_W` per axis"; no commit in `82651e8..HEAD` touches the spike or §2.4;
  `budget.rs:262` still says "no `DEMAND_MARGIN_W`, no hysteresis".
- **Suggested upstream fix:** the `ALREADY_MERGED` short-circuit ends in `bd close` — an
  irreversible tracker mutation on one unverifiable agent boolean. Cheap hardening, in
  `closeOnlyPrompt` itself (which already runs in the integration worktree and has git): make it
  re-verify the claim and report `merged: false` instead of closing when the branch has no commits
  of its own. That turns an unverified assertion into a two-agent agreement without giving the
  coordinator shell access.

### 11. Two of three blocker beads were filed malformed, in two different ways
- `fw-fanctrl-loop-6ma.1` (filed by the merge agent for `zct`'s red gate): `issue_type=task`,
  `--parent` the epic, **a dependency edge on `fw-fanctrl-loop-eyi`**, and **no `blocker` label**.
  Every clause of the contract's "bare `blocker` label, nothing else" rule was violated at once.
  As filed it was reachable as work, entangled in the dependency graph, and would have blocked
  epic closure permanently. It needed `--force` to close because of the dep it should never have had.
- `fw-fanctrl-loop-7e9` (filed by `j6s`'s implementer): correctly unparented, but also **no
  `blocker` label**. Harmless here only because the absent label is compensated by the structural
  membership test dropping it as OUT-OF-TREE.
- `fw-fanctrl-loop-bwt` was the only correctly-formed one.
- **Note:** the label-only rule is stated in all three filing prompts and in the self-filing
  implementer's dispatch text, in bold, with the durak-9rj → durak-hgr.18 loop as the cautionary
  tale — and agents still got it wrong 2 times out of 3. Prose in the prompt is evidently not
  sufficient; this wants a post-filing verification step (the coordinator already has the bead id
  in `RESULT.blockerBead`, so a mechanical "strip any parent/deps, ensure the label" dispatch
  after filing would close it deterministically).
- **Also:** `bd update <id> --label <l>` does not exist in bd 1.2.2 (`unknown flag: --label`), so
  repairing a mislabelled bead after the fact is not a one-liner.

### 12. The recurring-pattern detector never fired, across three runs and 50 minors that plainly cluster
- **What happened:** run 3's ledger carries **50 `minor (deferred)` lines across 22 tasks** and
  **zero `Recurring minor:` lines**. The whole-epic reviewer clustered the same corpus by hand and
  found three classes that each meet the ≥3-distinct-task threshold several times over:
  unfailable assertions (8 instances / 8 tasks), stale doc comments orphaned by scope fences
  (5 / 5), and `clippy -D warnings` red for most of the epic (4 / 4).
- **Cause (likely, not proven):** `noteRecurrence()` keys on `minorSignature()`, which lowercases,
  strips quoting, and replaces hashes/paths/digits — but nothing else. Two reviewers describing the
  same defect in their own prose produce different signatures, and the doc says so explicitly
  ("it does not try to cluster paraphrases"). The measured corpus here is exactly that: one class,
  N phrasings. So the detector is behaving as designed and the design does not fit the data — the
  threshold is reachable only when the *same pipeline* emits near-identical text (its original
  motivating case, a defect reported once per merge), not when N reviewers each describe one smell.
- **Consequence:** the reviewer had to do the clustering itself, twice (runs 2 and 3), and in run 2
  it reached for a wrong mechanical explanation to account for the silence (item 9). A detector that
  never fires trains its readers to explain away its silence.
- **Suggested upstream fix:** either say plainly in the ledger that clustering is verbatim-only and
  that a human/reviewer pass is still required, or cluster with a cheap embedding/LLM pass at
  Finish over the accumulated minors instead of at write time. The current middle ground reads as
  a working detector and is not one for reviewer-authored text.

### 13. `Sweep:` mislabels a non-test command's output as test counts
- **What happened:** the sweep line reads `07b31dd — 0 passed, 0 failed, 0 errors, 0 skipped;
  failing: none; command: cargo clippy --all-targets -- -D warnings`, and the sweep agent had to
  append its own disclaimer ("`cargo clippy` is a lint/build check, not a test runner").
- **Why it matters:** `0 passed` is the exact shape `sweepPrompt`'s own measurement-validity floor
  tells the agent to treat as `MEASUREMENT INVALID` ("a passed count near zero for a suite known to
  be large"). A clean clippy run and a catastrophically-failed test collection render identically.
  In run 2 the sweep DID report `MEASUREMENT INVALID`, correctly, for a different reason — so
  across the run the same field carried both meanings.
- **Suggested upstream fix:** `sweepPrompt`'s report shape hardcodes a test-runner vocabulary.
  Either let the declared sweep also declare its *kind* (test vs lint vs build), or make the
  required line `<tip> — <verdict: PASS|FAIL|INVALID> — <free-form detail>` and let the command's
  own idiom fill the detail.

### 14. A ledger append was refused by the safety classifier again — the elided retry saved it
- **What happened:** `ledger-append:merge:fw-fanctrl-loop-9it` was "blocked by safety classifier"
  (surfaced as the run's single `agents_error`). The run's own `Metrics: ledger-check ok ·
  append-failed 0 · append-retried 1` shows `appendLedger()`'s one retry with the free text elided
  landed the line. **This is issue #5 defect 9's fix working exactly as designed**, and worth
  recording as a success rather than a defect: the mechanism was added after a measured loss and it
  paid for itself here.
- **Residual:** the classifier refusal is still reported to the operator as a hard `agents_error`
  with a "SECURITY WARNING: This subagent performed actions that may violate security policy"
  banner (run 1 carried the same banner for the same label), which reads far more alarming than
  "one ledger line was rewritten without its prose". Nothing in the run summary connects the error
  to the successful retry; only the Metrics line does.

### 8. `defaultTestPathspecs` needed widening again once the plan was readable
- **What happened:** after fixing (4), the materialised plan showed task 2
  (`fw-fanctrl-loop-blm`) creates `tests/fixtures/fanctrl/*` — fixture data consumed by later
  tests, outside `src/**`. Resumed with `testPaths: ["src/**/*.rs", "tests/**"]`.
- **Note:** this is a second-order consequence of (4): the correct pathspec list for a project
  is not knowable until the plan exists, but `testPaths` must be supplied at launch. Not a
  defect — worth noting that the contract asks for this value one step before the information
  that determines it.

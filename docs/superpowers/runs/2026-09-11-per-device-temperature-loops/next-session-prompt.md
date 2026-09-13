# Next-session prompt — design roast extension round for per-device temperature loops

Paste everything below the line into a fresh Claude Code session started from
`/var/home/alepar/AleCode/fw-fan-quiet/.claude/worktrees/fw-fanctrl-loop`.

---

Resume the super-auto run `2026-09-11-per-device-temperature-loops` at `phase: roast-design`.
The run is paused at the design-roast cap (round 3 of 3 still Blocking). Your job this session:
(1) fix every confirmed finding of design roast round 3, (2) run the one capped-Blocking
**extension round** of super-roast in design mode, (3) record the result and stop — do not
proceed to super-code, whatever the verdict. I approve the fix pass and the extension round now;
autonomous within that scope.

## Where everything is

- Run worktree (all writes go here, on branch `super-auto/per-device-temperature-loops`):
  `/var/home/alepar/AleCode/fw-fan-quiet/.claude/worktrees/fw-fanctrl-loop/.worktrees/super-auto-per-device-temperature-loops`
- Run dir: `docs/superpowers/runs/2026-09-11-per-device-temperature-loops/` inside that worktree, containing
  `run.md` (state; `roastDesignRound: 3`, `roastDesignCapped:` line), the spec
  `2026-09-11-per-device-temperature-loops-design.md` (revision 3), `task-tree-settled.md`
  (regenerated from bd), the coverage ledger `per-device-temperature-loops-coverage-ledger.md`
  (append "Design roast 3 dispositions" like the d1/d2 sections), and the three roast reports
  `…-roast-design-{1,2,3}.md`. Last commit on the branch: `7bbe761`.
- Bead tree: root epic `fw-fanctrl-loop-eb9`, leaves `eb9.1`–`eb9.16`, sweep `eb9.17`
  (`bd list --label sp:fw-fanctrl-loop-eb9 --status all --limit 0`). Every leaf body was rewritten
  wholesale against rev3 last session — keep doing wholesale rewrites (`bd update <id> --description`
  replaces the whole field; preserve the `blocked-by …` lines and the `Files:` line). Never append
  "amendment" paragraphs; roast round 2 flagged that as Blocking.
- Scratchpad from the previous session (may be gone; recreate if missing):
  `/tmp/claude-1000/-var-home-alepar-AleCode-fw-fan-quiet--claude-worktrees-fw-fanctrl-loop/6e6f1052-b05b-421f-abbd-edfbee8e089d/scratchpad/`
  with `super-roast-engine.mjs` (the Workflow script), `assemble-design-roast.py <iteration> <prior_report_path>`
  (renders pointer-style prompts into `roast-prompts-design-N/` and `roast-args-design-N.json`),
  `extract-design-report.py <run_id> <iteration>` (pulls `reportMarkdown` out of the workflow journal
  into the run dir), `beads-rev3.py` (the wholesale bead-rewrite pattern), `record-roast-d2.py`
  (tree regeneration + ledger + run.md pattern). If the scratchpad is gone, the prompt sources are the
  super-roast skill files under
  `~/.claude/plugins/cache/superpowers-alepar/superpowers/6.3.0-alepar3.8/skills/super-roast/`
  (`scout-prompts-design.md`, `judge-seat-prompts.md`, `dedupe-prompt.md`, `reporter-prompt.md`,
  `triage-prompt.md`, `super-roast-workflow.md` for the engine).

## Harness rules that bit last session

- Always `git -C <absolute worktree path> …`; the shell cwd silently resets to the parent worktree
  and a bare `git` there once merged onto the wrong branch. Never touch branch
  `worktree-fw-fanctrl-loop` or `epic-fw-fanctrl-loop-6ma-integration` this session.
- The worktree guard refuses compound commands / heredocs / variables that mention `git`. Use the
  Write tool for scripts and plain single commands; `python3 script.py` is fine.
- `bd` works from the run worktree. `bd show <id> --json` returns a list.
- No real-hardware step may be automated (the 30-min gaming check and the VR/VRAM label spike are
  the user's, parked in run.md). `sudo.txt` is gitignored; not needed this session.

## The round-3 findings to fix (report: `…-roast-design-3.md`, 4 Blocking / 15 Should-fix, all confirmed, no rejections to re-litigate)

Blocking:
1. **Tracking lock-up, third incarnation** (§2.3 steps 3–4). `thermal := min(thermal, cap)` while
   err < 0 reduces to `min(thermal, shadow)`, and the shadow falls on a draw dip regardless of
   temperature, so a dip while the group is still above T\* drags the thermal candidate to a value
   the device was never thermally limited at; recovery at the integral rate. The tie rule (→ Shadow)
   also makes `DeviceUnreachable` unreachable.
2. **Per-group boxcar seeding** (§2.2, eb9.2). The carried §2.2 seeds from fw-fanctrl's single argmax
   `movingAverageTemperature`, which has no per-group equivalent, so `cpu_group_ma`/`gpu_group_ma` at
   entry/resume/mismatch-clear is unspecified; and `view_changed` is listed as a reseed trigger, which
   the carried design forbids (view change = `set_interval` only).
3. **Sim 4 bar vs the GPU dead zone** (§4, eb9.11). The 300 MHz zero-gain crossing costs ≈ 535–714 s
   at the default GPU gains, so "settles within 3 λ = 810 s" after a 90 s dead time is unsatisfiable
   by the spec's own arithmetic.
4. **`verify_lock` re-scope** (§2.5 step 5 vs step 3). No pairing between the clock sample and the
   command it is scored against, and `DOWN_RATE_MHZ 105 > VERIFY_CLOCK_SLACK_MHZ 30`, so a compliant
   card scores strikes during any ratchet.

Should-fix (fix all; each is one or two sentences in the report — read it): tick contract has no
`dt` / post-resume dt bound; GPU HOT guard keys on the die while tracking keys on the EC group, so
after a ratchet to the floor the cap follows `max` back up untracked; a setpoint-driven hot spell
(T\* re-derived down) ratchets the thermal candidate although the load never changed;
`t_star_last_good` is unkeyed/untimestamped while its neighbours are keyed; `Uncontrollable` entry/exit
steps the cap for any device whose thermal candidate is below its shadow; the stateless plausibility
gate diverges from fw-fanctrl's unfiltered `x > 0` average and latches `EC MISMATCH` on a > 110 °C
label; `DrawUnavailable` no longer belongs in the up-blocking anti-windup set (thermal-only cap is
live); under Bypass the shadow's `err ≥ 0` rise gate still applies against a frozen T\*; the
`DrawUnavailable` dwell expiry and `gpu_shadow_enabled = false` both restore the full ~947 MHz
plateau with no sim; `Clamp(Max)` is unreachable under the tie rule (both candidates share the
clamp); GPU τ = 35 s has no derivation; a plausible-but-stuck uncontrollable label latches
`Uncontrollable` for the session; labels outside the fixed eight-label list are classified
uncontrollable (firmware/kernel rename); `T*_floor` undefined when the plausible uncontrollable set
is empty (both labels on one F75303 chip); the `cpu_hot_c` sanitiser floor `gpu_hot_c + 1` is wrong
(only ≥ 82 is needed).

## Mechanism recommendation for finding 1 (decide, then apply consistently)

Three rounds have each replaced the tracking rule and each produced a new lock-up at the same seam:
a min-selector between a slow PI and a fast draw-tracking shadow always needs a tracking rule, and
every tracking rule couples the PI state to the draw signal. Prefer the structural fix over a fourth
tracking rule:

- **Drop the min-selector.** The thermal PI owns the cap. The shadow becomes a **rate limiter on the
  applied cap's rise above draw**, not a competing candidate: `cap_applied = min(thermal,
  shadow_ceiling)` where `shadow_ceiling` rises from `draw + headroom` at the rise slew and is
  re-anchored to `draw + headroom` only when the PI output is *below* it (so it never binds while
  the PI is regulating). No tracking rule at all; the PI's own directional anti-windup at
  `[floor, max]` is the only integrator gate. A never-hot device has thermal = max and its cap is
  the ceiling ramping above draw; a hot device has thermal < ceiling and the PI is in charge; a
  draw dip lowers the ceiling toward `draw + headroom` but never touches the PI state, so recovery
  after the dip is at the rise slew, not the integral rate.
- If you keep a two-candidate form instead, the roast's own fix-shape hint is the minimum: track only
  on the hand-over tick (the first tick with err < 0 while Shadow was selected), never continuously;
  gate the shadow's fall on err ≥ 0 like its rise; and give `Selected`/`Hold` a tie rule that
  reports `Clamp(Floor)`/`Clamp(Max)` when the thermal candidate is at a bound.

Either way: rewrite §2.3 steps 2–4 and 7, the defaults table, the "GPU dead zone" paragraph and
the §4 unit bars as one coherent text; then rewrite eb9.3 and eb9.4 bodies wholesale (and eb9.7,
eb9.11, eb9.14, eb9.16 where they cite tracking, tie rules or `Clamp(Max)`).

For finding 3, set sim 4's cold-start bar from the mechanism (e.g. "GPU HOT does not trip; group
settles within ±1 °C within 3 λ **after the dead-zone crossing**, crossing time ≤ headroom /
(Kc·T/Ti·|e|) + θ_eff") or shrink the GPU headroom default to 150 MHz and recompute; do not leave a
bar the spec's own arithmetic refutes. For finding 2, seed each group boxcar from its own
instantaneous group max at entry/resume/mismatch-clear (or leave it `None` until filled and cite
eb9.3's absent-group rule), and delete `view_changed` from the reseed list in §2.2 and eb9.2. For
finding 4, pair each verification sample with the command in force when the sample was taken
(one-tick lag) and score `reported ≤ commanded_prev + slack + one ratchet step` or suspend scoring
for `VERIFY_SETTLE_TICKS` after every lock change.

## Procedure

1. `bd prime`; read `run.md`, the spec, `task-tree-settled.md`, and `…-roast-design-3.md` in full.
2. Apply the fixes: spec (a `spec-rev4.py` with exact-string replacements, asserting each `old`
   exists), then bead bodies wholesale, then regenerate `task-tree-settled.md` from bd, then append
   "Design roast 3 dispositions (d3-01..d3-19)" to the ledger with one line per finding
   (`id · severity · location · applied · what → beads`), then in `run.md` set
   `roastDesignRound: 4`, replace the `roastDesignCapped:` line with
   `roastDesignExtension: round 4 (capped-Blocking extension) launched <date>`, and commit with
   `git -C <worktree> commit` (message style: `docs(design): apply design roast 3 — spec revision 4, …`;
   end with the Co-Authored-By trailer the harness prints).
3. Run the extension round: `assemble-design-roast.py 4 <path to …-roast-design-3.md>`; edit
   `roast-args-design-4.json`'s `priorReport` to point at report 3 and name reports 1–2 as siblings;
   keep `"iteration": 4` in args but the report header must read `iteration: post-cap extension`
   (edit the reporter's `{{ITERATION}}` value in the args to the literal string
   `post-cap extension` — the engine passes it through). Launch with the `Workflow` tool:
   `scriptPath` = the engine, `args` = the JSON contents. Wait for the task notification
   (≈ 35–45 min, ~90–130 agents, ~7–9 M subagent tokens).
4. `extract-design-report.py <run_id> 4`, add a `roast-design:` bullet to `run.md`, commit the
   report. Then **stop and report to the user**: verdict, delta line, the Blocking list if any.
   - Should-fix/clean: say the design loop is done and super-code (phase 3) is next; do not start it.
   - Still Blocking: record `roastDesignCapped:` again and say the run is `capped-blocking`;
     do not run another round.

## Do not

- Do not launch super-code, merge anything, or touch the integration branch.
- Do not re-litigate the three reports' Rejected sections.
- Do not automate any hardware step.

You independently verify ONE review finding about a design spec. Confirm it only if it
is real and material. **Material means material against the spec's stated requirements,
contract, and scope — not against an imagined stricter system.** A behavior the spec explicitly
accepts as a tradeoff (with or without mitigation) is not a gap; a demand for guarantees
or scale the spec explicitly bounds away is not a gap. Do not rubber-stamp, and do not
reject reflexively — judge on the merits.

## Spec
DESIGN MODE. The spec is `/var/home/alepar/AleCode/bazerame-fans/.claude/worktrees/fw-fanctrl-loop/.worktrees/super-auto-per-device-temperature-loops/docs/superpowers/runs/2026-09-11-per-device-temperature-loops/2026-09-11-per-device-temperature-loops-design.md`; its settled task tree is `/var/home/alepar/AleCode/bazerame-fans/.claude/worktrees/fw-fanctrl-loop/.worktrees/super-auto-per-device-temperature-loops/docs/superpowers/runs/2026-09-11-per-device-temperature-loops/task-tree-settled.md` (part of the artifact). The artifact is the design spec `/var/home/alepar/AleCode/bazerame-fans/.claude/worktrees/fw-fanctrl-loop/.worktrees/super-auto-per-device-temperature-loops/docs/superpowers/runs/2026-09-11-per-device-temperature-loops/2026-09-11-per-device-temperature-loops-design.md` TOGETHER WITH its settled task tree `/var/home/alepar/AleCode/bazerame-fans/.claude/worktrees/fw-fanctrl-loop/.worktrees/super-auto-per-device-temperature-loops/docs/superpowers/runs/2026-09-11-per-device-temperature-loops/task-tree-settled.md` (18 beads: root epic fw-fanctrl-loop-eb9, 16 leaves, an integration sweep; every bead carries owns:/consumes:/blocked-by lines and an acceptance) — the tree is the decomposition the spec will be executed as, so a gap in the tree against the spec, or a spec decision the tree cannot deliver, is in scope. The spec supersedes §2.4/§2.5 of `/var/home/alepar/AleCode/bazerame-fans/.claude/worktrees/fw-fanctrl-loop/.worktrees/super-auto-per-device-temperature-loops/docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md` (its §2.2/§2.6/§2.8/§2.9/§3.4 carry over and may be read for context); the seed that settled the user decisions is `/var/home/alepar/AleCode/bazerame-fans/.claude/worktrees/fw-fanctrl-loop/.worktrees/super-auto-per-device-temperature-loops/docs/superpowers/specs/2026-09-11-per-device-temperature-loops-seed.md`; the code the tree will change is the Rust crate at `/var/home/alepar/AleCode/bazerame-fans/.claude/worktrees/fw-fanctrl-loop/.worktrees/super-auto-per-device-temperature-loops` (branch super-auto/per-device-temperature-loops, base epic-fw-fanctrl-loop-6ma-integration). Field evidence the spec rests on: telemetry runs `/var/lib/bazerame-fans/telemetry/run-1789067819.jsonl` and `run-1789139478.jsonl` (readable). Decided by the user and NOT up for re-litigation unless you show them unworkable: budget/split deleted; per-device EC groups; GPU driven by clock lock (LUT deleted); shadow cap with override control; halt = hold; no trim with a live curve; per-device step test; the 30-min hardware acceptance and the VR/VRAM label spike are the user's and are parked. Shell discipline: absolute paths; `cd /var/home/alepar/AleCode/bazerame-fans/.claude/worktrees/fw-fanctrl-loop/.worktrees/super-auto-per-device-temperature-loops` as the first statement of every call; plain separate commands (this harness refuses compound constructs and heredocs that mention git as "too complex to verify" — a re-spell-and-retry, never a permission denial). Do NOT modify any file.

Verify this finding only against the spec/diff named above — never against a different file, spec, or PR you happen to find nearby.

## The finding to verify (JSON)
{{FINDING_JSON}}

Use whatever fields are present (typically `claim`, `location`, `category`, `external`,
`kind`, `evidence`, `suggestedSeverity`). Treat `suggestedSeverity` as a hint only, never
authoritative — your own severity judgment is independent of it.

## Your seat: REPRODUCE
Build the strongest concrete demonstration that the finding is real. For a GAP: locate
where the spec should address it, show that it doesn't, then walk the concrete failure
story — step by step, each step cited to spec text — until it contradicts a **stated
requirement** of the spec. For an UNVERIFIED-ASSUMPTION: show where the design leans on
it and what breaks if it is false. CONFIRM only if the demonstration completes
end-to-end, including the final step against a stated requirement — a mechanism that
"can happen" but never contradicts anything the spec promises is not a completed
demonstration. If the demonstration breaks down, REJECT and say exactly where it broke.

## Grounding rule (MANDATORY, all seats)
- If the claim (or a premise your CONFIRM relies on) depends on a fact about the outside
  world — a library/API capability, a scaling limit, a default behavior, a
  version-specific detail — you MUST verify it with **actual web research**
  (WebSearch / WebFetch — fetch the page; you cannot spawn the deep-research skill as a
  dispatched agent). Do NOT confirm or reject an external-fact claim from memory — those
  facts vary by version/config and memory is exactly where reviews go confidently wrong.
  - **A CONFIRM of an external-fact finding REQUIRES a resolved citation** (a real URL
    you fetched + the supporting quote) in `evidence`. No citation = you may NOT CONFIRM it.
  - If research finds nothing conclusive either way, return **UNVERIFIED** — do not
    silently REJECT a possibly-real risk just because you couldn't source it.
- Internal/structural claims are verified against the **spec text** itself.

## Severity (use for CONFIRM; see Output contract for REJECT/UNVERIFIED)
- **Blocking:** if unaddressed, the change is likely to be wrong, lose data, or fail its
  core purpose — must fix before proceeding.
- **Should-fix:** significant risk or rework, address before/soon after merge.
- **Nit:** real but low-impact.
- **FYI:** context/observation, no action required.

Never use `blocker`, `major`, `minor`, `BLOCK`, `REVISE`, or `PASS` — those vocabularies are
retired.

## Output contract (exact — return one JSON object matching this shape, no prose outside it)
`{"verdict": "CONFIRM" | "REJECT" | "UNVERIFIED", "severity": "Blocking" | "Should-fix" | "Nit" | "FYI", "evidence": "<string>"}`

- `verdict: "CONFIRM"` — the finding is real and material. `severity` is your judgment from
  the scale above. `evidence` carries the demonstration/evidence (REQUIRED resolved URL +
  quote if the claim is external-fact).
- `verdict: "REJECT"` — the finding is not real or not material. `evidence` explains why.
  `severity` is required by the schema but carries no meaning here — set it to `"FYI"`.
- `verdict: "UNVERIFIED"` — use ONLY for external-fact claims you could not ground either
  way after real research. `evidence` states what you could not confirm/refute and the
  cheapest way a human could. These are routed to a human, not dropped. Never use
  UNVERIFIED to dodge an internal/structural finding. `severity` is required by the schema
  but carries no meaning here — set it to `"FYI"`.

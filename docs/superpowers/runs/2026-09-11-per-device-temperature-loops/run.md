# super-auto run — 2026-09-11-per-device-temperature-loops

flags: planOneShot=false skipPlanRoast=false skipCodeRoast=false autonomous=true
phase: super-code

idea: Replace the scalar power budget and CPU/GPU split in bazerame-fans with per-device temperature loops: one shared setpoint T* from the fan target and the live fw-fanctrl curve, a CPU PI on max(cpu@4c, apu) driving the ryzenadj sustained watts cap, a GPU PI on max(gpu_vr, gpu_vram, gpu_amb, gpu_temp) driving the GPU max-clock lock directly (LUT, sweep and watts inner loop deleted), a per-device shadow cap (draw + headroom, fast rise / slow fall) combined with the thermal cap through a min selector with tracking (override control) so no device ever runs uncapped and load jumps become ramps of seconds, a slow outer RPM trim PI on T* for Mode B and drift, and a per-device step test for gains. The full settled design, field evidence, sensor mapping, acceptance sketch and open items are in the committed seed file docs/superpowers/specs/2026-09-11-per-device-temperature-loops-seed.md (commit 859c878) — hand it to super-design as the root brainstorm's context and treat its settled items as decided unless a roast overturns them. Base branch: epic-fw-fanctrl-loop-6ma-integration (tip 859c878), NOT master — cut the run branch from it and merge back into it; the integration worktree for that branch is /var/home/alepar/AleCode/bazerame-fans/.claude/worktrees/fw-fanctrl-loop/.worktrees/epic-fw-fanctrl-loop-6ma-integration. Flags: planOneShot=false (interactive design), skipPlanRoast=false, skipCodeRoast=false, autonomous=true (the user is needed at the top-split gate, then unattended to the merge decision). Gates for super-code: gate `cargo test`, sweep `cargo clippy --all-targets -- -D warnings`, concurrency 4, hotFileCap 3. Shell discipline for every dispatched agent: absolute paths and `cd <abs>` first in every command; the harness worktree guard refuses compound commands and heredocs that mention git — use the Write tool for scripts and plain single commands. No real-hardware step may be automated: the acceptance's 30-min gaming check is the user's, parked in the report.
branch: super-auto/per-device-temperature-loops
base: epic-fw-fanctrl-loop-6ma-integration
spec: 2026-09-11-per-device-temperature-loops-design.md
epic: fw-fanctrl-loop-eb9
parked:
- kind: escalation · source: design roast 1 (§2.4 Held plant gain: EC-autofan curve ~0 RPM/°C at 67–73 °C vs the fixed 78 RPM/°C) · mitigation applied in spec rev2 / eb9.16: the slope schedule is retained (Kc × slope_ref/max(slope_at(T*), slope_ref) clamped to [0.25, 1], 0.25× when no curve resolves a slope) · RESOLVED by design roast 2 (d2-21): sim 6 (eb9.14) is the EC-autofan leg, written against λ_eff with the 0.25× floor engaged, and gates eb9.16 through the integration sweep · no user action needed
- kind: escalation · source: promotion review (design) · VR/VRAM label-to-label mapping between framework_tool and hwmon is a hardware load-test spike (spec Facts); taken on faith in eb9.1 (groups are max, no cross-group ambiguity by name) · owner: the user, on this machine
approvals:
- top-split (2026-09-11, human): epic fw-fanctrl-loop-eb9; children eb9.1 LEAF, eb9.2 LEAF, eb9.3 LEAF, eb9.4 LEAF, eb9.5 LEAF (demoted-by-session), eb9.6 LEAF, eb9.7 LEAF (demoted-by-session), eb9.8 LEAF (demoted-by-session), eb9.9 LEAF (demoted-by-session, TUI split out), eb9.10 LEAF, eb9.11 LEAF (demoted-by-session, sims 5-8 split out), eb9.12 LEAF, eb9.13 LEAF, eb9.14 LEAF
coverage-round-1:
- requirements: 16 · mapped: 16 · unmapped: 0 () · R-new: 2 (R17 calibration freeze of both loops; R18 DeviceLoop seed/resync API)
- reviewers: 3/3 valid (opus, input-bounded) · raw 40 · deduped 21 (incl. 5 flag-sweep) · applied 18 (auto) · accepted 3 (auto) · rejected 1 (auto: already applied)
- tree changes: edges eb9.7<-eb9.4, eb9.13<-eb9.9, eb9.7<-eb9.9, eb9.5<-eb9.6, eb9.12<-eb9.11, eb9.12<-eb9.14, eb9.8<-eb9.7; new leaves eb9.15 (GPU HOT max ratchet, split from eb9.7) and eb9.16 (Held driver, split from eb9.5); descriptions amended on eb9.3, eb9.5, eb9.6, eb9.7, eb9.9, eb9.10, eb9.11, eb9.12, eb9.14
- ledger: per-device-temperature-loops-coverage-ledger.md (c1-01..c1-21)
coverage-round-2:
- requirements: 18 · mapped: 18 · unmapped: 0 () · R-new: 0
- reviewers: 3/3 valid · raw 14 · deduped 13 · applied 13 (auto) · rejected 0 · count 22 → 13 (shrinking; all identities trace to round-1 fixes — check 9 doing its job, not scope widening)
- tree changes: edges eb9.9<-eb9.16, eb9.13<-eb9.16, eb9.4<-eb9.6, eb9.15<-eb9.6, eb9.7<-eb9.16, eb9.12<-eb9.15; descriptions amended on eb9.3, eb9.4, eb9.7, eb9.9, eb9.10, eb9.12, eb9.14, eb9.16
- integration sweep: eb9.17 (blocks on eb9.1..eb9.16)
- ledger: per-device-temperature-loops-coverage-ledger.md (c2-01..c2-13)
roastDesignRound: 4
roast-design:
- 2026-09-11-per-device-temperature-loops-roast-design-1.md (Blocking, 32 confirmed: 3 Blocking / 27 Should-fix / 2 Nit; 8 scouts, 153 raw -> 86 deduped, 61 panels, judge completion 100%, not degraded) — all 32 confirmed applied as spec revision 2 + bead amendments (ledger: Design roast 1 dispositions d1-01..d1-32); escalation mitigated and parked
- 2026-09-11-per-device-temperature-loops-roast-design-2.md (Blocking, 27 confirmed: 8 Blocking / 18 Should-fix / 1 Nit; delta 16 new (4 B) · 0 carried · 22 resolved · 10 regressed (6 B); 9 scouts, 77 raw -> 41 deduped, 38 panels, judge completion 100%, not degraded) — all 27 applied as spec revision 3 (parking/band replaced by hot-only tracking; jump rule deleted; directional Held anti-windup; entry in Held; ThermalMode) + every bead body rewritten wholesale (ledger: Design roast 2 dispositions d2-01..d2-27)
- 2026-09-11-per-device-temperature-loops-roast-design-3.md (Blocking, 19 confirmed: 4 Blocking / 15 Should-fix; delta 14 new (2 B) · 0 carried · 22 resolved · 5 regressed (2 B); 9 scouts, 42 raw -> 29 deduped, 25 panels, judge completion 100%, not degraded) — all 19 applied as spec revision 4 and wholesale bead descriptions (ledger d3-01..d3-19); authorized extension follows
- 2026-09-11-per-device-temperature-loops-roast-design-4.md (clean (0 nits) [low coverage]; 0 new confirmed (0 Blocking) · 0 carried (0 Blocking) · 19 resolved · 0 regressed (0 Blocking); 9/9 scouts completed, 0 raw/deduped candidates, no judge panels, no failed stages) — extension completed; low-coverage qualifier is required by reporter policy for zero raw findings on a non-trivial artifact, not by a dead scout or incomplete panel.
roastDesignExtension: round 4 (capped-Blocking extension) launched 2026-09-11

- Extension authorization (2026-09-11): user adopted next-session-prompt.md; fix all round-3 confirmations, run exactly one post-cap extension, record verdict and stop before super-code regardless of verdict. Codex manual super-roast fallback uses fresh OpenAI GPT scouts and differentiated judges; no Workflow tool is available.

roastDesignExtensionStatus: complete — clean (0 nits) [low coverage]; stopped after the authorized extension
next: execute epic fw-fanctrl-loop-eb9 through the super-code task/review/integration queue
stopReason: none — user explicitly started super-code after reviewing the extension outcome

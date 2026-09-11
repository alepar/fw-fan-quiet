# super-auto run — 2026-09-11-per-device-temperature-loops

flags: planOneShot=false skipPlanRoast=false skipCodeRoast=false autonomous=true
phase: design

idea: Replace the scalar power budget and CPU/GPU split in bazerame-fans with per-device temperature loops: one shared setpoint T* from the fan target and the live fw-fanctrl curve, a CPU PI on max(cpu@4c, apu) driving the ryzenadj sustained watts cap, a GPU PI on max(gpu_vr, gpu_vram, gpu_amb, gpu_temp) driving the GPU max-clock lock directly (LUT, sweep and watts inner loop deleted), a per-device shadow cap (draw + headroom, fast rise / slow fall) combined with the thermal cap through a min selector with tracking (override control) so no device ever runs uncapped and load jumps become ramps of seconds, a slow outer RPM trim PI on T* for Mode B and drift, and a per-device step test for gains. The full settled design, field evidence, sensor mapping, acceptance sketch and open items are in the committed seed file docs/superpowers/specs/2026-09-11-per-device-temperature-loops-seed.md (commit 859c878) — hand it to super-design as the root brainstorm's context and treat its settled items as decided unless a roast overturns them. Base branch: epic-fw-fanctrl-loop-6ma-integration (tip 859c878), NOT master — cut the run branch from it and merge back into it; the integration worktree for that branch is /var/home/alepar/AleCode/bazerame-fans/.claude/worktrees/fw-fanctrl-loop/.worktrees/epic-fw-fanctrl-loop-6ma-integration. Flags: planOneShot=false (interactive design), skipPlanRoast=false, skipCodeRoast=false, autonomous=true (the user is needed at the top-split gate, then unattended to the merge decision). Gates for super-code: gate `cargo test`, sweep `cargo clippy --all-targets -- -D warnings`, concurrency 4, hotFileCap 3. Shell discipline for every dispatched agent: absolute paths and `cd <abs>` first in every command; the harness worktree guard refuses compound commands and heredocs that mention git — use the Write tool for scripts and plain single commands. No real-hardware step may be automated: the acceptance's 30-min gaming check is the user's, parked in the report.
branch: super-auto/per-device-temperature-loops
base: epic-fw-fanctrl-loop-6ma-integration
spec: 2026-09-11-per-device-temperature-loops-design.md
epic: fw-fanctrl-loop-eb9
parked:
- kind: escalation · source: promotion review (design) · VR/VRAM label-to-label mapping between framework_tool and hwmon is a hardware load-test spike (spec Facts); taken on faith in eb9.1 (groups are max, no cross-group ambiguity by name) · owner: the user, on this machine
approvals:
- top-split (2026-09-11, human): epic fw-fanctrl-loop-eb9; children eb9.1 LEAF, eb9.2 LEAF, eb9.3 LEAF, eb9.4 LEAF, eb9.5 LEAF (demoted-by-session), eb9.6 LEAF, eb9.7 LEAF (demoted-by-session), eb9.8 LEAF (demoted-by-session), eb9.9 LEAF (demoted-by-session, TUI split out), eb9.10 LEAF, eb9.11 LEAF (demoted-by-session, sims 5-8 split out), eb9.12 LEAF, eb9.13 LEAF, eb9.14 LEAF
coverage-round-1:
- requirements: 16 · mapped: 16 · unmapped: 0 () · R-new: 2 (R17 calibration freeze of both loops; R18 DeviceLoop seed/resync API)
- reviewers: 3/3 valid (opus, input-bounded) · raw 40 · deduped 21 (incl. 5 flag-sweep) · applied 18 (auto) · accepted 3 (auto) · rejected 1 (auto: already applied)
- tree changes: edges eb9.7<-eb9.4, eb9.13<-eb9.9, eb9.7<-eb9.9, eb9.5<-eb9.6, eb9.12<-eb9.11, eb9.12<-eb9.14, eb9.8<-eb9.7; new leaves eb9.15 (GPU HOT max ratchet, split from eb9.7) and eb9.16 (Held driver, split from eb9.5); descriptions amended on eb9.3, eb9.5, eb9.6, eb9.7, eb9.9, eb9.10, eb9.11, eb9.12, eb9.14
- ledger: per-device-temperature-loops-coverage-ledger.md (c1-01..c1-21)

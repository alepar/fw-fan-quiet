# Task 23 report: README + docs (fw-fanctrl-loop-7ij)

## What I implemented

Rewrote the parts of `README.md` the brief scoped (Calibration walkthrough, Auto mode,
Safety model, Configuration table, opening paragraph, `--state-file` flag description,
File locations, bottom Status note), added a pointer note to the top of
`docs/research/03-control.md`, and flipped the spec's status in
`docs/superpowers/specs/INDEX.md` to `implemented`. No source files were touched — this
was a docs-only task.

Everything was written from the actual code in this integration worktree, not from the
design doc's prose, per the brief's "read the code, not this plan" instruction:

- Config keys/defaults: read directly from `src/config.rs`'s `Config`/`LedConfig` struct
  definitions and their `Default` impls (and the clamp defaults `CPU_MAX_W`/`GPU_MAX_W`
  from `control/allocator.rs`, confirmed against `config.rs`'s own clamp tests, which is
  where `cpu_max_w`'s 54.0 and `gpu_max_w`'s 100.0 come from).
- `StatusFlag` variants: read from the `enum StatusFlag` and its `as_str`/`flag_span`
  (`ui/view.rs`) definitions in `src/control/controller.rs`.
- Calibration flow (phases, durations, skip/abort reasons, constants): read from
  `src/calib/step.rs` (`StepTest`, `SETTLE_CAP_SAMPLES`, `STEP_W`, `EC_MAX_ABORT_C`, etc.)
  and `src/calib/lut_sweep.rs` (`SWEEP_CLOCKS`, `PIN_STREAK`, `PIN_UTIL_MIN_PCT`,
  `PIN_CLOCK_SLACK_MHZ`).
- Auto-mode cascade/arbiter/modes: read from `src/control/mode.rs` (`Arbiter::decide`, the
  row table, entry hysteresis, reconciliation) and `src/control/budget.rs`'s module docs,
  cross-checked against design doc §2.4-§2.7 for the numbers `mode.rs` itself only states
  as bare constants (e.g. the 5 °C feasibility margin, the 2 %/°C steepness threshold).
- Guards / NVMe safety line: read from `src/control/guards.rs`'s module doc, which carries
  the exact 2026-09-08 measurement (67→80 °C in 30 s, EC max falling 74→69 °C) the brief
  asked to be stated plainly.
- Socket read-only-by-construction claim: read from `src/fanctrl/client.rs`'s module doc
  and the `PrintCommand` enum (exactly two variants).
- TUI header format and key bindings: read from `src/ui/view.rs` (the `mode {mode} · T*
  ... · budget ...` format string and its own tests) and `src/model.rs` (`on_manual_key`,
  the step/clamp constants).
- Telemetry fields: read from `src/telemetry.rs`'s `Record` enum and its module doc
  (schema version 2's field additions).

## An important finding, recorded rather than papered over

`src/control/controller.rs`'s own tests (e.g.
`auto_allocate_decision_carries_zero_demand_arbiter_defaults`, comment: *"The arbiter
(design §2.5) isn't wired up yet"*) and a grep for `Arbiter`/`Budget` usage inside
`controller.rs` confirm that **the arbiter (`control/mode.rs`) and the budget PI
(`control/budget.rs`) are not yet called from the live `on_sample`/`on_auto_sample` path**.
Pressing `a` today still drives the older scalar-budget-split allocator (which, per that
same test file, is presently a stub that just holds at the floor). The modules the README
now documents (arbiter, budget PI, `DutyRpmTable`, the socket client, the guards) all
exist, compile, and are unit-tested standalone — the names, thresholds and behavior I
documented are all real and code-verified — but wiring them into the controller's live
Auto-mode loop is `fw-fanctrl-loop-j6s` ("Controller loop integration") and
`fw-fanctrl-loop-438` ("Controller hooks: warm-start, refinement, calibration"), both
still open siblings of this task in the epic, not dependencies of it.

Per the brief's own explicit deliverable list ("Auto mode — the cascade, the modes, and a
flags table", using the real `StatusFlag`/`LoopMode` names, several of which are
`#[allow(dead_code)]` because *no call site raises them yet*) and the acceptance criterion
("every `StatusFlag` variant appears in the flags table"), the README is clearly meant to
document the target design using the real code-level names/types now that they exist —
not to wait for the remaining wiring tasks. I wrote it that way, but added an explicit
caveat to the bottom "Status" paragraph so a reader isn't misled into thinking `a` drives
the full cascade on this branch today. Flagging this here so the reviewer (and whoever
picks up `fw-fanctrl-loop-j6s`) has it explicitly, rather than it being buried in a diff.

## Verification (acceptance criteria)

**1. Every config key in `config.rs` appears in the README table and vice versa** (script
output, both directions matched exactly):

```
Config.rs struct fields:      README top-level config table keys:
fan_target_rpm                fan_target_rpm
cpu_floor_w                   cpu_floor_w
gpu_floor_mhz                 gpu_floor_mhz
fast_limit_mw                 fast_limit_mw
cpu_max_w                     cpu_max_w
gpu_max_w                     gpu_max_w
gpu_hot_c                     gpu_hot_c
nvme_hot_c                    nvme_hot_c
leds                          fanctrl_socket
fanctrl_socket                leds
```

Same check for the nested `LedConfig`/`[leds]` table (I added this table proactively since
`leds` is itself a config key whose sub-fields weren't in the old README at all):

```
LedConfig struct fields:      README leds table keys:
enabled                       enabled
cpu_port                      cpu_port
gpu_port                      gpu_port
brightness                    brightness
flip_time                     flip_time
cpu_flip_watts                cpu_flip_watts
gpu_flip_watts                gpu_flip_watts
```

**2. Every `StatusFlag` variant appears in the flags table** (script output — the 13
`as_str()` names, sorted):

```
curve_invalid, ec_mismatch, fanctrl_lost, gpu_hot, limit_not_sticking, not_calibrated,
nvme_hot, readback_blind, resumed, sensor_lost, steep_curve, target_unreachable,
thermal_emergency
```

All 13 appear as rows in the README's Auto mode flags table (`THERMAL EMERGENCY`,
`SENSOR LOST`, `TARGET UNREACHABLE`, `LIMIT-SLIP!`, `NOT CALIBRATED`, `CURVE INVALID`,
`EC MISMATCH`, `FANCTRL LOST`, `GPU HOT`, `NVME HOT`, `resumed`, `STEEP CURVE`,
`READBACK BLIND`) — one row per variant, no extras, no omissions. Table row order follows
`ui/view.rs`'s `render_priority` (the actual live severity ordering shown in the TUI header
when several flags compete for one line), not `controller.rs`'s separate `flag_severity`
classifier — the two disagree on `NvmeHot`'s tier (Info vs. the yellow/Warning it's
actually rendered as), and `ui/view.rs`'s own comment says the spec (yellow, alongside
`GpuHot`) wins since `flag_severity` isn't wired into any rendering. I followed the live
renderer.

**3. No README mention of matrix/model/trim/RLS (the deleted subsystems) remains** —
`grep -in "matrix\|model\|\btrim\b\|\brls\b" README.md` output after the rewrite:

```
71:| `gpu_hot_c` | ... See Safety model below |
72:| `nvme_hot_c` | ... see Safety model below |
74:| `leds` | ... `[leds]` table for the optional LED matrix wattage display |
122:   A first-order-plus-dead-time model is fitted to both the EC-temperature response and
177:| `NVME HOT` | ... see Safety model |
194:## Safety model
```

All six hits are legitimate, unrelated uses, not residue of the deleted subsystems: "Safety
model" is the section name the design doc (§3.5) itself mandates; "LED matrix" is the
physical Framework 16 LED Matrix input-module hardware (a real, undeleted feature, distinct
noun from the deleted calibration matrix phase); "first-order-plus-dead-time model" is the
FOPDT fit the new step-test calibration performs (`calib::fopdt::fit_fopdt`) — itself part
of what this task was asked to document. Zero occurrences of "trim" or "RLS" of any kind
remain (the old integrator-trim / online-RLS-adaptation subsystem and its `online_rls`
config key are both fully gone from the README — confirmed separately: `grep online_rls
README.md` also finds nothing).

## Files changed

- `README.md` — opening paragraph, `--state-file` row, Configuration table (dropped
  `online_rls`; added `cpu_max_w`, `gpu_max_w`, `gpu_hot_c`, `nvme_hot_c`,
  `fanctrl_socket`, `leds` + a new `[leds]` sub-table), Calibration walkthrough (LUT sweep
  + step test), Auto mode (cascade diagram, TempLoop/RpmLoop/Released mode table, full
  13-row flags table), Telemetry (updated field list), Safety model (fw-fanctrl-owns-fans,
  read-only-by-construction, guards incl. the NVMe reporting-only line + its measurement,
  read-back), File locations, and the bottom Status paragraph (added the controller-wiring
  caveat above).
- `docs/research/03-control.md` — added a pointer note at the top directing readers to
  `docs/research/05-fw-fanctrl-loop.md` and the 2026-09-07 design doc, and stating which
  sections of this older research doc (the model form, the calibration matrix) no longer
  describe the shipped system.
- `docs/superpowers/specs/INDEX.md` — status column for the 2026-09-07 design row changed
  from `designed (coverage 2/2 rounds, 23 leaves)` to `implemented`, per the brief's
  explicit instruction.

## Self-review

- Re-read the whole rewritten README top to bottom for internal consistency (key bindings,
  clamp ranges, and CLI flag defaults were cross-checked against `src/model.rs` and
  `src/main.rs` and were already accurate in the old README, so those sections were left
  alone beyond the two edits noted above).
- Did not touch the LED-matrix top-of-README hardware description, Requirements,
  `ryzen_smu` caveat, Build & run, or Keys sections — out of this task's scope and already
  accurate.
- Did not attempt to fix the controller-wiring gap found above — that's `j6s`/`438`'s job,
  not this task's; I only made sure the README doesn't silently overclaim about it.

## Concerns

- The "Auto mode" section documents the target architecture using real, code-verified
  names, but (as recorded above) that architecture is not yet wired into the live
  controller loop on this branch. I judged this the correct call per the brief's explicit
  deliverable list and the acceptance criteria (which require documenting flags/modes that
  literally have no call site yet), and mitigated the risk with an explicit caveat in the
  Status paragraph — but flagging it for the reviewer's own judgment call.

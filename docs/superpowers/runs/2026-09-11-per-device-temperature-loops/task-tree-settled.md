# Settled task tree — epic fw-fanctrl-loop-eb9 (after coverage rounds 1–2)

## fw-fanctrl-loop-eb9 [epic] Per-device temperature loops (replace the budget split)
blocking deps: []
Root epic for the 2026-09-11 super-auto run (branch super-auto/per-device-temperature-loops, base epic-fw-fanctrl-loop-6ma-integration). Spec: docs/superpowers/runs/2026-09-11-per-device-temperature-loops/2026-09-11-per-device-temperature-loops-design.md. Goal: the fan target is met by regulating each device's own EC temperature group toward one shared setpoint T*; no scalar budget, no CPU/GPU split; bumpless Auto entry; shadow caps with override control; T* source replacing the Mode A/B arbiter; GPU clock driven directly (LUT deleted); per-device step test. Run completion = this epic closed. Hardware acceptance (30-min gaming check) and the VR/VRAM label spike are the user's and are parked in run.md, not beads.

## fw-fanctrl-loop-eb9.1 [task] EC sensor groups: cpu_group_c / gpu_group_c on EcReading
blocking deps: []
Spec §2.1. Add the fixed CPU group {cpu@4c, apu_f75303@4d} and GPU group {gpu_vr, gpu_vram, gpu_amb, gpu_temp@40} label sets to sensors/ec.rs; EcReading gains cpu_group_c/gpu_group_c = max over the group's positive readings, None when none; is_controllable() becomes group().is_some(); argmax/all unchanged. Update the hwmon fixtures so cros_ec_dgpu_on presents a gpu_* sensor as the argmax at ~83 C under load (synthetic values allowed; a hardware re-capture is not automatable) and cros_ec_idle keeps ambient as argmax. Acceptance: unit tests for both groups incl. dGPU-unpowered -> gpu_group_c None; existing ec tests green.
owns: the group label sets, EcReading.cpu_group_c / gpu_group_c.
consumes: nothing new.
Files: src/sensors/ec.rs, tests/fixtures/hwmon/*
Note (promotion review): the VR/VRAM label-to-label mapping between framework_tool and hwmon is taken on faith from the seed's table; the spec's load-test spike is hardware-only and parked for the user in run.md. Because each group is a max, a swap INSIDE the GPU group changes nothing; only a cross-group swap would matter, and none of the eight labels is ambiguous across groups by name.

## fw-fanctrl-loop-eb9.2 [task] Per-group EC boxcars in EcAverage (replica)
blocking deps: ['fw-fanctrl-loop-eb9.1']
Spec §2.2. EcAverage/replica keeps its argmax boxcar (reconciliation unchanged) and adds one boxcar per group over the same ma_interval; all three seed/reseed/invalidate together on auto entry, view_changed, resume and EC MISMATCH exactly as §2.2 of the prior design; a group reading None clears its own boxcar and reports None. Public API: cpu_group_ma()/gpu_group_ma() -> Option<f64>. Acceptance: unit tests for seeding, interval change, None handling; the existing assert_ec_ma_tracks_emulator test still green.
owns: the per-group boxcar API on EcAverage.
consumes: EcReading.cpu_group_c / gpu_group_c.
blocked-by fw-fanctrl-loop-eb9.1: consumes EcReading.cpu_group_c/gpu_group_c
Files: src/sensors/ec.rs

## fw-fanctrl-loop-eb9.3 [task] DeviceLoop core: types, thermal PI, clamps, hold, tick contract
blocking deps: []
Spec §2.3 steps 1, 2, 4 (selector shape), 5, 6 — the UNBLOCKING ARTIFACT for every consumer. New control/device_loop.rs: DeviceLoop generic over unit (W | MHz), Gains{kc, ti_s} with the spec's CPU/GPU defaults, Hold{None, Shadow, Clamp, GroupUnavailable}, Selected{Thermal, Shadow, Floor, Max}, DeviceDecision, tick(input) -> (cap, DeviceDecision); velocity-form thermal PI at PI_PERIOD_S=5 with output clamp [floor,max], directional conditional integration (the sat_dir rule moved from gpu_pid.rs), quantise (0.5 W) / slew (105 MHz/s, floors win) moved from allocator/gpu_pid; the pinned tests (CPU 2 W margin; GPU 105 MHz + util floor) moved from allocator::demand. The shadow candidate is a stub in this bead: selector = thermal candidate only, Selected never Shadow (the shadow bead fills it in). Pure, no I/O. Acceptance: step response on a first-order plant within 1 % with <=5 % overshoot at defaults for both units; clamp hold and directional unwind tests (carry the gpu_pid unwind test over); GroupUnavailable holds the last applied cap, and when no cap was ever applied (group None from the first tick) it reports max with hold GroupUnavailable so the caller can skip the actuator write; after seed(x, y) the first tick applies x (thermal) without a kick; after resync_error() the next tick's proportional term is zero regardless of the setpoint jump. The "first-order plant" in the acceptance is a local test stub inside device_loop.rs's test module (no dependency on the two-node plant bead).
owns: DeviceLoop, Gains, Hold, Selected, DeviceDecision, the tick contract, the seed/resync API — seed(thermal_at, shadow_at) for bumpless entry and resync_error() (resets e_prev only; no proportional kick; shadow untouched) for T* re-derivation — and the pinned tests.
consumes: nothing.
Files: src/control/device_loop.rs, src/control/mod.rs

## fw-fanctrl-loop-eb9.4 [task] Shadow cap candidate, min selector and tracking (override control)
blocking deps: ['fw-fanctrl-loop-eb9.3', 'fw-fanctrl-loop-eb9.6']
Spec §2.3 steps 3–4 in full. In device_loop.rs: shadow = draw + headroom; rises one headroom step per sample while pinned and group_c < t_star - band; falls toward draw + headroom at shadow_fall_rate; clamped [floor,max]; cap = min(thermal, shadow); Selected names the binding candidate; tracking sets the unselected candidate's state to cap each tick; the thermal integrator updates only when Thermal was selected on the previous PI tick. Config-driven parameters shadow_headroom / shadow_band_c / shadow_fall_rate per device with the spec defaults (CPU 10 W, 3 C, 0.33 W/s; GPU 300 MHz, 3 C, 10 MHz/s). Acceptance: 38 -> 100 W ramp completes in ~6 s while pinned and cool; a scene dip drops the shadow by one headroom in 30 s; descent-from-max test: after 300 s with the shadow binding, the thermal candidate is within one integral step of the cap; never below floor; after seed(t, s) with the shadow binding, the first tick applies min(t, s) with Selected::Shadow and no step relative to draw + headroom; resync_error() leaves the shadow candidate and its fall state untouched.
owns: the shadow candidate, the selector and tracking, the shadow candidate's USE of the shadow Config keys (the key declarations, defaults and sanitisers are eb9.6's).
consumes: DeviceLoop tick contract, Gains, Hold, Selected.
blocked-by fw-fanctrl-loop-eb9.3: consumes the DeviceLoop tick contract and Selected/Hold types
blocked-by fw-fanctrl-loop-eb9.6: consumes the shadow Config keys (declarations, defaults, sanitisers)
Files: src/control/device_loop.rs

## fw-fanctrl-loop-eb9.5 [task] TStarSource states: Curve / Held / Released, curve derivation, reconciliation, T* at the API boundary
blocking deps: ['fw-fanctrl-loop-eb9.3', 'fw-fanctrl-loop-eb9.6']
Spec §2.4. New control/tstar.rs replacing the mode.rs arbiter: states Curve (view fresh + curve valid + replica reconciled, ENTRY_HYSTERESIS_S 15 s at 1 Hz), Held (T* frozen at the last good value; the RPM PI that drives it, the feasibility/steepness flags and DeviceUnreachable are the sibling Held-driver bead), Released; T* re-derivation on view_changed / fan-target change with resync on both loops; §2.6 reconciliation moved here; t_star_last_good exposed at the TStarSource API boundary (loaded/persisted by the controller bead). Acceptance: unit tests per transition at 1 Hz, no proportional kick on re-derivation, Held freezes T* at the last good value when no driver is attached; mode.rs's still-relevant state-machine tests ported.
owns: TStarSource, its states and transitions, the Curve derivation, the T* persistence field's meaning at the API boundary, the Held slot the driver bead fills.
consumes: Hold (device_loop), FanctrlView + curve + DutyRpmTable (existing), EcAverage reconciliation verdict, EcReading.all.
blocked-by fw-fanctrl-loop-eb9.3: consumes the Hold type
consumes (added, coverage r1): the DeviceLoop seed/resync API (eb9.3) for the no-kick re-derivation; PersistedState.t_star_last_good (eb9.6) via the controller's load/persist wiring (eb9.7).
blocked-by fw-fanctrl-loop-eb9.6: consumes PersistedState.t_star_last_good (field shape)
Files: src/control/tstar.rs, src/control/mode.rs (logic moves out), src/control/mod.rs

## fw-fanctrl-loop-eb9.6 [task] Config keys and PersistedState migration for two loops
blocking deps: ['fw-fanctrl-loop-eb9.3']
Spec §2.8 and the Config half of §2.3. Config: shadow_headroom_{cpu_w,gpu_mhz}, shadow_band_c, shadow_fall_rate_{cpu,gpu}, cpu_gains / gpu_gains overrides, gpu_max_mhz (default 3090, sanitised to [gpu_floor_mhz, 3090]); remove gpu_max_w (old file loads with one warning). PersistedState: remove lut; loop_gains -> cpu_gains/gpu_gains: Option<Gains>; warm_start values -> WarmStartEntry{cpu_cap_w, gpu_lock_mhz} (bare-number values dropped with a log line); t_star_last_good: Option<f64>; validated() covers the new fields. Acceptance: old-file migration tests (lut ignored, loop_gains ignored, bare warm-start dropped), sanitizer tests per new key, save/load round trip.
owns: the new Config keys, PersistedState's new shape, WarmStartEntry and the WarmStart::key(strategy, duty, on_ac) form (round-trip tested).
consumes: Gains (device_loop).
blocked-by fw-fanctrl-loop-eb9.3: consumes the Gains type
Files: src/config.rs, src/state.rs

## fw-fanctrl-loop-eb9.7 [task] Controller wiring: AutoState = TStarSource + two DeviceLoops
blocking deps: ['fw-fanctrl-loop-eb9.15', 'fw-fanctrl-loop-eb9.16', 'fw-fanctrl-loop-eb9.2', 'fw-fanctrl-loop-eb9.3', 'fw-fanctrl-loop-eb9.4', 'fw-fanctrl-loop-eb9.5', 'fw-fanctrl-loop-eb9.6', 'fw-fanctrl-loop-eb9.9']
Spec §2.5. AutoState holds tstar, cpu: DeviceLoop<W>, gpu: DeviceLoop<Mhz>, the replica, steady window, warm start. on_auto_sample: replica tick -> groups; tstar.tick -> T*; guards: GPU HOT ratchets the GPU loop's max down by DOWN_RATE per tick to no lower than floor; cpu.tick / gpu.tick -> caps; write through the existing actuator paths (read-back, verify_lock, stickiness watchdog, Mismatch re-write, shutdown fences, reassert). Bumpless entry: thermal integrators seed at max(floor, draw) (or the warm-start pair), shadows at draw + headroom. Warm start records (cpu_cap, gpu_lock) when both groups are within 1 C of T* AND the fan is within the steady window's tolerance for STEADY_WINDOW_N (spec §2.5 step 6; not recorded while the fan is outside tolerance). Remove Budget/allocator/mode plumbing from controller.rs (the files are deleted by the sweep bead). Manual/Monitor/calibration/release/quit/emergency-release BEHAVIOUR unchanged, WIRING changed: each path now addresses the two loops instead of the budget. Also owns: removing the NOT CALIBRATED gate on SetAuto(true) (controller.rs; the flag stays informational, raised when either device has no fitted gains) and the GPU HOT ratchet on the GPU loop's max at DOWN_RATE_MHZ = 105 per allocation tick with symmetric recovery (spec §2.5). Acceptance: controller tests for entry seeding (no draw change > pin_margin on the first tick), GPU HOT ratchet wired to the max, warm-start record/seed (and NOT recorded while the fan is outside tolerance), resume reassert; the existing guard/watchdog/fence tests re-pointed at the two loops and green — verify_lock Mismatch re-write on both actuators, stickiness watchdog trip -> Released, shutdown fence and stock restore per device, emergency release; caps held constant across a simulated calibration run and no cap step on unfreeze/reseed (R17); t_star_last_good loaded at Auto entry and written back on change; per-device draw values reaching each tick; table refinement invoked from the steady window; a GPU whose group reads None from the first tick (dGPU unpowered) seeds at max and has NO NVML lock written until its group appears; wiring_sweep exhaustive matches updated (incl. the emergency path).
owns: AutoState composition, the TStarSource tick input plumbing (fan_target, the raw fan readings behind FAN_SMOOTH_N, EcReading.all, and each loop's PREVIOUS-tick Hold/Selected — tstar.tick runs before cpu.tick/gpu.tick in a sample, so its Hold feedback is one sample lagged by construction), the decision-line emission call site in on_auto_sample (fields per the v3 schema; the Budget* values are no longer computed or emitted), the per-device draw inputs (cpu_pkg_w; gpu_sm_mhz + gpu_util_pct) passed from the sample into each loop's tick, the steady-window duty<->RPM table refinement (unchanged from today), loading t_star_last_good into TStarSource at Auto entry and persisting it on change, the calibration freeze/unfreeze of both loops (hold applied caps for the run's duration, reseed on exit with no cap step), the Auto-entry gate (no calibration required), the wiring of the GPU HOT max ratchet into the GPU loop's max (the ratchet rule itself is the helper bead), the write path from caps to actuators, warm-start pair semantics.
consumes: DeviceLoop tick, TStarSource, EcAverage groups, Config keys, PersistedState shape.
blocked-by fw-fanctrl-loop-eb9.3: consumes the DeviceLoop tick contract
blocked-by fw-fanctrl-loop-eb9.5: consumes TStarSource
blocked-by fw-fanctrl-loop-eb9.2: consumes the per-group boxcar API
blocked-by fw-fanctrl-loop-eb9.6: consumes the new Config keys and PersistedState shape
blocked-by fw-fanctrl-loop-eb9.4: consumes the shadow candidate and its seeding (bumpless entry seeds shadows at draw + headroom)
blocked-by fw-fanctrl-loop-eb9.9: consumes the v3 decision schema (the emission call site fills it)
consumes (added, coverage r1): the DeviceLoop seed/resync API (eb9.3).
blocked-by fw-fanctrl-loop-eb9.15: consumes the MaxRatchet helper
blocked-by fw-fanctrl-loop-eb9.16: consumes the Held driver's tick inputs and the lagged-Hold convention
Files: src/control/controller.rs, src/integration_tests.rs

## fw-fanctrl-loop-eb9.8 [task] Per-device step test (CPU watts step, GPU clock step), settle gate, gains save
blocking deps: ['fw-fanctrl-loop-eb9.2', 'fw-fanctrl-loop-eb9.3', 'fw-fanctrl-loop-eb9.6', 'fw-fanctrl-loop-eb9.7']
Spec §2.6. calib/step.rs becomes a per-device step test run twice by the runner: settle gate (both groups flat 0.5 C/60 s, fans within 150 RPM/20 s, argmax controllable, 600 s cap, skip reason names the failing condition and duration); CPU step +15 W with GPU held, fit the CPU group (W/C), reject if the GPU group moved > 1 C; GPU step +500 MHz with CPU held, fit the GPU group (MHz/C), reject if the CPU group moved > 1 C; save cpu_gains/gpu_gains; rejected fit -> defaults for that device with a log line; NOT CALIBRATED stays informational (the Auto-entry gate removal is owned by the controller bead, spec §2.5). Fit window FIT_WINDOW_S = 360 s per device step after settle (spec §2.6). lut_sweep.rs is no longer invoked (deleted by the sweep bead). Acceptance: runner tests on the fakes for each gate outcome, both fits, cross-term rejection, persistence of per-device gains.
owns: the step-test procedure, settle gate thresholds, gains persistence call.
consumes: Gains (device_loop), PersistedState cpu_gains/gpu_gains, EcAverage groups.
blocked-by fw-fanctrl-loop-eb9.3: consumes the Gains type
blocked-by fw-fanctrl-loop-eb9.6: consumes PersistedState.cpu_gains/gpu_gains
blocked-by fw-fanctrl-loop-eb9.2: consumes the per-group boxcar API
blocked-by fw-fanctrl-loop-eb9.7: consumes the controller's calibration freeze/unfreeze of both loops (caps held for the run's duration)
Files: src/calib/step.rs, src/calib/runner.rs, src/calib/fopdt.rs

## fw-fanctrl-loop-eb9.9 [task] Telemetry v3 decision line and sample groups
blocking deps: ['fw-fanctrl-loop-eb9.1', 'fw-fanctrl-loop-eb9.16', 'fw-fanctrl-loop-eb9.3', 'fw-fanctrl-loop-eb9.4', 'fw-fanctrl-loop-eb9.5']
Spec §2.9. decision line schema v3: t_star, tstar_state, per device {group_c, err_c, thermal, shadow, cap, selected, hold}, cpu_limit_w, gpu_max_mhz, flags; the Budget*/pi_target_w/alloc_*/demand_*/freeze fields are NOT removed here (this bead lands before the controller is rewired and must keep the crate compiling): they are marked deprecated and are removed by the deletion sweep (eb9.12) once the emission call site (eb9.7) stops filling them. sample line adds cpu_group_c/gpu_group_c. Acceptance: the crate compiles and every_telemetry_field_is_populated is green with the v3 fields added and the deprecated fields still populated by the old call site; a decision-line round-trip test with every Selected/Hold value populated from a real DeviceDecision.
owns: the v3 decision schema, the sample-line group fields, and the README/docs telemetry-schema section updated field-by-field to v3.
consumes: DeviceDecision (device_loop), TStarSource state (tstar), EcReading groups.
blocked-by fw-fanctrl-loop-eb9.3: consumes DeviceDecision
blocked-by fw-fanctrl-loop-eb9.5: consumes the TStarSource state enum
blocked-by fw-fanctrl-loop-eb9.4: consumes Selected::Shadow / Hold::Shadow semantics (structurally dead before the shadow bead)
blocked-by fw-fanctrl-loop-eb9.1: consumes EcReading.cpu_group_c/gpu_group_c (the sample-line group fields)
blocked-by fw-fanctrl-loop-eb9.16: consumes the feasibility/SteepCurve/DeviceUnreachable flag set (serialised into the v3 flags field)
Files: src/telemetry.rs, src/types.rs, README.md (telemetry section)

## fw-fanctrl-loop-eb9.10 [task] ChainedPlant: second thermal node per device with cross-coupling, clock->heat GPU model
blocking deps: []
Spec §4 (Simulation). test_support/plant.rs: a CPU node (watts -> CPU group, tau~35 s) and a GPU node (clock -> GPU group with the 40 s gpu_vr tail), cross-coupling 0.1 C/C each way, the fanctrl emulator unchanged; the GPU model per spec §4: draw_w = load_level x P_full(clock) with P_full the September full-load table (1197->49.3, 1402->53.5, 1612->64.2, 1807->75.9, 1995->90.8, 2143->99.4 W; flat 100 W to 3090; linear to (1000, 45) below), reported SM clock = lock when load_level >= 0.9 else lock x load_level, heat = draw_w x 0.8 C/W; CPU draw = min(cap, cpu_load_w), heat x 0.8 C/W; scriptable load levels per device and a load step. Acceptance: each injected fault is observable at the sample boundary in a unit test; open-loop step tests on each node match the stated tau/dead time within 10 %; cross term visible at the stated magnitude.
owns: the two-node plant, its load scripting API, and a fault-injection API: dGPU power-off (GPU group reads None), fan-reading outage (fan_valid false), EC-invalid / stale fanctrl view (drives EC MISMATCH and Released), and a scriptable GPU die temperature output (gpu_temp_c, the NVML reading the GPU HOT guard 88/86 keys on — distinct from the EC gpu_* group) so a sim can trip GPU HOT deliberately, each observable at the replica/sample boundary.
consumes: nothing (test support).
Files: src/test_support/plant.rs, src/test_support/mod.rs

## fw-fanctrl-loop-eb9.11 [task] Closed-loop acceptance sims 1–4 and the two-loop sim helpers
blocking deps: ['fw-fanctrl-loop-eb9.10', 'fw-fanctrl-loop-eb9.4', 'fw-fanctrl-loop-eb9.7']
Spec §4 sims 1–4 in control/sim_tests.rs on the two-node plant, plus the shared helpers (build_controller for two loops, run_ticks with per-device load scripts, trace rows with per-device columns): CPU-heavy/light-GPU; GPU-heavy/light-CPU; both heavy; GPU load step (ramp ~6 s, no crest > target+250, CPU cap unchanged). Sims 5–8 are the sibling bead that consumes these helpers. Bar: fans within +-150 RPM of target >= 90 % of a 30-min converged window where stated.
owns: the two-loop sim helpers and sims 1–4, and the removal of the ClockWattsLut / gpu_watts_lut sims from sim_tests.rs (the deletion sweep does not touch sim_tests.rs).
consumes: the wired controller, the two-node plant, the shadow cap behaviour.
blocked-by fw-fanctrl-loop-eb9.7: consumes the wired on_auto_sample
blocked-by fw-fanctrl-loop-eb9.10: consumes the two-node plant
blocked-by fw-fanctrl-loop-eb9.4: consumes the shadow cap behaviour
Files: src/control/sim_tests.rs

## fw-fanctrl-loop-eb9.12 [task] Delete the budget/allocator/arbiter/LUT machinery and sweep docs
blocking deps: ['fw-fanctrl-loop-eb9.11', 'fw-fanctrl-loop-eb9.13', 'fw-fanctrl-loop-eb9.14', 'fw-fanctrl-loop-eb9.15', 'fw-fanctrl-loop-eb9.7', 'fw-fanctrl-loop-eb9.8', 'fw-fanctrl-loop-eb9.9']
Spec §3. Delete src/control/budget.rs, allocator.rs, spike_antiwindup.rs, mode.rs, lut.rs, calib/lut_sweep.rs, the watts inner loop in gpu_pid.rs (file removed once DeviceLoop owns its rules), DEMAND_MARGIN_W_*, Freeze::DemandLimited, Budget* telemetry fields, Config::gpu_max_w, gpu_share_override (symbol, any config/telemetry surface), the deprecated Budget*/pi_target_w/alloc_*/demand_*/freeze telemetry fields, PersistedState::lut/loop_gains remnants, gpu_watts_lut fixtures (the ClockWattsLut sims in sim_tests.rs are removed by the sims bead eb9.11). Sweep the repo for each deleted path AND basename, source and non-source (Cargo.lock, docs, README, research notes, design docs): every hit is removed or owned; mark §2.4/§2.5 of the 2026-09-07 design as superseded with a pointer to the new spec; README describes the two loops, T* source and shadow caps; docs/research cross-links. Acceptance: grep for each deleted basename returns only historical review/run documents; cargo test + clippy -D warnings green; README Status paragraph current.
owns: the deletions and the doc sweep.
consumes: the wired controller, calib and telemetry (nothing may still reference the deleted modules).
blocked-by fw-fanctrl-loop-eb9.7: consumes the controller no longer referencing budget/allocator/mode
blocked-by fw-fanctrl-loop-eb9.8: consumes the runner no longer referencing lut_sweep
blocked-by fw-fanctrl-loop-eb9.9: consumes the telemetry line no longer carrying Budget fields
blocked-by fw-fanctrl-loop-eb9.13: consumes the TUI no longer rendering the budget/allocation panel
blocked-by fw-fanctrl-loop-eb9.11: consumes sim_tests.rs no longer referencing ClockWattsLut, and the acceptance sims 1–4 green at the sweep gate
blocked-by fw-fanctrl-loop-eb9.14: consumes the acceptance sims 5–8 green at the sweep gate (all leaves precede the terminal sweep)
blocked-by fw-fanctrl-loop-eb9.15: consumes the MaxRatchet replacement for gpu_share_override
Files: src/control/*, src/calib/*, src/telemetry.rs, README.md, docs/**, Cargo.toml

## fw-fanctrl-loop-eb9.13 [task] TUI for two loops: T* + state, group temps, caps with binding candidate, hold, unreachable
blocking deps: ['fw-fanctrl-loop-eb9.16', 'fw-fanctrl-loop-eb9.3', 'fw-fanctrl-loop-eb9.4', 'fw-fanctrl-loop-eb9.5', 'fw-fanctrl-loop-eb9.9']
Spec §2.9 (TUI half). ui/view.rs + ui/model.rs: show T* and its TStarSource state, both group temperatures with their error, both caps with the binding candidate (T/S/F/M), the hold state, and the per-device unreachable flags; replace the budget/allocation panel; keys unchanged. Acceptance: every_status_flag_is_raised_and_rendered green; a snapshot test per new panel; the view renders with a None group (dGPU unpowered).
owns: the TUI panels for two loops.
consumes: DeviceDecision (device_loop), TStarSource state (tstar), the v3 ControlStatus fields.
blocked-by fw-fanctrl-loop-eb9.3: consumes DeviceDecision
blocked-by fw-fanctrl-loop-eb9.5: consumes the TStarSource state enum
blocked-by fw-fanctrl-loop-eb9.4: consumes Selected::Shadow / Hold::Shadow semantics
blocked-by fw-fanctrl-loop-eb9.9: consumes the v3 ControlStatus fields
blocked-by fw-fanctrl-loop-eb9.16: consumes the feasibility/SteepCurve/DeviceUnreachable flag set (rendered per device)
Files: src/ui/view.rs, src/ui/model.rs

## fw-fanctrl-loop-eb9.14 [task] Closed-loop acceptance sims 5–8: bumpless entry, curve loss, unreachable device, configuration smoke
blocking deps: ['fw-fanctrl-loop-eb9.11', 'fw-fanctrl-loop-eb9.16']
Spec §4 sims 5–8 in control/sim_tests.rs using the two-loop helpers: bumpless entry under steady heavy load (no draw change > pin_margin in 10 s, fans do not fall); curve loss mid-session -> Held -> fans back within +-150 RPM within 10 min -> curve return with no cap step; unreachable device (a GPU that cannot reach T*) never changes the CPU cap; configuration smoke: every TStarSource state, every Hold value AND every Selected value (Thermal, Shadow, Floor, Max — Floor/Max via a GPU-HOT-ratchet-to-floor leg and the unreachable device at max) reached behaviourally across the sims (asserted on effects/status, not on self-pushed vectors), driving GroupUnavailable and Released through the plant's fault-injection API (needs: fw-fanctrl-loop-eb9.10), plus an "Auto entry with gpu_group_c None" leg (no GPU lock written, CPU loop regulates alone) and the GPU-HOT-to-floor leg driven by the plant's scriptable GPU die temperature (needs: fw-fanctrl-loop-eb9.10). Acceptance: all four green with the stated bars.
owns: sims 5–8.
consumes: the two-loop sim helpers (sims 1–4 bead), the wired controller, the two-node plant.
blocked-by fw-fanctrl-loop-eb9.11: consumes the two-loop sim helpers
blocked-by fw-fanctrl-loop-eb9.16: consumes the Held driver (RPM PI) for the curve-loss sim and the feasibility/unreachable flags for the smoke
Files: src/control/sim_tests.rs

## fw-fanctrl-loop-eb9.15 [task] GPU HOT max ratchet helper for the GPU loop's max (105 MHz/tick down, symmetric recovery)
blocking deps: ['fw-fanctrl-loop-eb9.3', 'fw-fanctrl-loop-eb9.6']
Spec §2.5 step 3 (split out of the controller bead by coverage round 1). A pure MaxRatchet helper in control/guards.rs: while GPU HOT is active the GPU loop's max steps down by DOWN_RATE_MHZ = 105 per allocation tick to no lower than gpu_floor_mhz; when the guard clears it steps back up at the same rate to gpu_max_mhz; idempotent per tick; unit-tested for the down ramp, the floor stop, the symmetric recovery and a guard that flaps. Replaces gpu_share_override (watts) which the deletion sweep removes.
owns: the GPU max ratchet rule and its constants.
consumes: the GPU HOT guard state (existing guards.rs), gpu_floor_mhz/gpu_max_mhz (config).
blocked-by fw-fanctrl-loop-eb9.3: consumes the GPU DeviceLoop's max bound semantics
blocked-by fw-fanctrl-loop-eb9.6: consumes gpu_max_mhz / gpu_floor_mhz bounds
Files: src/control/guards.rs

## fw-fanctrl-loop-eb9.16 [task] TStarSource Held driver: RPM PI on T*, feasibility/steepness flags, per-device DeviceUnreachable
blocking deps: ['fw-fanctrl-loop-eb9.3', 'fw-fanctrl-loop-eb9.5']
Spec §2.4 second half (split out of the TStarSource bead by coverage round 1). In control/tstar.rs: the Held-state driver — a slow velocity-form RPM PI (err_rpm = fan_target - fan_smoothed, FAN_SMOOTH_N = 5 tail mean, Kc ≈ 4.1e-4 °C/RPM, Ti = 35 s, PI_PERIOD_S = 5, output step bounded to 0.5 °C per PI tick, clamp [max(uncontrollable)+5, gpu_hot_c-2], integrator holds when both loops report Hold::Clamp at max or at floor); §2.7 feasibility (TargetUnreachable low/high) and SteepCurve (informational); the per-device DeviceUnreachable flag (at max & under T* for BOUND_HOLD → informational; at floor & over T* → real). Acceptance: unit tests for the RPM PI clamp/hold/step bound (the hold rule reads the PREVIOUS tick's per-device Hold — one-sample lag — asserted explicitly), each feasibility flag, each DeviceUnreachable case; the Held sim (sims 5–8 bead) consumes this.
owns: the RPM PI, the feasibility/steepness flags, DeviceUnreachable, and the export of those flag values on the TStarSource decision for the v3 schema and the TUI.
consumes: TStarSource states and the Held entry/exit (the states bead), both loops' Hold (eb9.3), EcReading.all, fan readings.
blocked-by fw-fanctrl-loop-eb9.5: consumes the TStarSource state machine and its Held slot
blocked-by fw-fanctrl-loop-eb9.3: consumes the Hold type
Files: src/control/tstar.rs

## fw-fanctrl-loop-eb9.17 [task] Integration sweep: per-device temperature loops end to end
blocking deps: ['fw-fanctrl-loop-eb9.1', 'fw-fanctrl-loop-eb9.10', 'fw-fanctrl-loop-eb9.11', 'fw-fanctrl-loop-eb9.12', 'fw-fanctrl-loop-eb9.13', 'fw-fanctrl-loop-eb9.14', 'fw-fanctrl-loop-eb9.15', 'fw-fanctrl-loop-eb9.16', 'fw-fanctrl-loop-eb9.2', 'fw-fanctrl-loop-eb9.3', 'fw-fanctrl-loop-eb9.4', 'fw-fanctrl-loop-eb9.5', 'fw-fanctrl-loop-eb9.6', 'fw-fanctrl-loop-eb9.7', 'fw-fanctrl-loop-eb9.8', 'fw-fanctrl-loop-eb9.9']
Root integration sweep (coverage loop terminal join). On the merged tree: (1) walk the goal's main flows end to end on the fakes/plant — engage Auto from a fresh state.json under a steady two-device load (bumpless), hold at T* in Curve, lose the curve -> Held -> regain, GPU HOT trip and recovery, calibration run (freeze, both steps, gains saved) and a simulated daemon restart reloading cpu_gains/gpu_gains/warm-start pair/t_star_last_good; (2) sweep for unwired values: every Config key read somewhere, every StatusFlag raised and rendered, every v3 telemetry field populated from live data, every DeviceDecision/TStar field reaching the TUI, no reference to any deleted module/symbol outside historical docs; (3) add the integration tests no per-bead test covers (sampler -> controller -> telemetry with real types; a full on_command/on_sample session on the fakes); fix small gaps inline, file blockers for large ones. Acceptance: the flows above pass; the unwired-sweep checklist recorded in the spec's Post-Implementation Notes with zero open items or a filed blocker per item; cargo test + clippy -D warnings green.
blocked-by fw-fanctrl-loop-eb9.1: consumes all leaves (integration sweep)
blocked-by fw-fanctrl-loop-eb9.2: consumes all leaves (integration sweep)
blocked-by fw-fanctrl-loop-eb9.3: consumes all leaves (integration sweep)
blocked-by fw-fanctrl-loop-eb9.4: consumes all leaves (integration sweep)
blocked-by fw-fanctrl-loop-eb9.5: consumes all leaves (integration sweep)
blocked-by fw-fanctrl-loop-eb9.6: consumes all leaves (integration sweep)
blocked-by fw-fanctrl-loop-eb9.7: consumes all leaves (integration sweep)
blocked-by fw-fanctrl-loop-eb9.8: consumes all leaves (integration sweep)
blocked-by fw-fanctrl-loop-eb9.9: consumes all leaves (integration sweep)
blocked-by fw-fanctrl-loop-eb9.10: consumes all leaves (integration sweep)
blocked-by fw-fanctrl-loop-eb9.11: consumes all leaves (integration sweep)
blocked-by fw-fanctrl-loop-eb9.12: consumes all leaves (integration sweep)
blocked-by fw-fanctrl-loop-eb9.13: consumes all leaves (integration sweep)
blocked-by fw-fanctrl-loop-eb9.14: consumes all leaves (integration sweep)
blocked-by fw-fanctrl-loop-eb9.15: consumes all leaves (integration sweep)
blocked-by fw-fanctrl-loop-eb9.16: consumes all leaves (integration sweep)
Files: src/** (small inline fixes), src/integration_tests.rs, docs/superpowers/runs/2026-09-11-per-device-temperature-loops/*.md (notes)

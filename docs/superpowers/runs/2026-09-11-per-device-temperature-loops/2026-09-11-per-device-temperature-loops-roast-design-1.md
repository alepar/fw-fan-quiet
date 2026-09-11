---
super-roast verdict: Blocking (32 confirmed)
mode: design        iteration: 1 of 3
profile (assumed): Single-operator personal tooling with hardware side-effects: a Rust daemon on one Framework 16 laptop that writes CPU power limits via ryzenadj (sudo) and dGPU clock locks via NVML, and reads the EC through cros_ec/ectool. No network surface, no other users, no external data. Blast radius is the operator's own machine: a wrong cap or a stuck emergency release degrades performance or acoustics and, at worst, lets the hardware's own thermal protection (card slowdown 89 C / shutdown 92 C; EC trip points) take over. Rollback is a git revert and a daemon restart. Thermal-safety inversions, silent loss of control, control-loop instability (windup, chatter, starvation) and anything that defeats the hardware backstops are the material class; resilience/observability polish is low-value here.
inputs: docs/superpowers/runs/2026-09-11-per-device-temperature-loops/2026-09-11-per-device-temperature-loops-design.md + docs/superpowers/runs/2026-09-11-per-device-temperature-loops/task-tree-settled.md (epic fw-fanctrl-loop-eb9, 18 beads)
coverage: triage ok · 8 scouts ran (0 dead) · dedupe ok · 153 raw → 86 deduped → 60 panel + 25 spot (1 promoted to panel) · judge completion 100% · remainder-capped: 0
independence: same-family (Claude) — seat-differentiated panel
seat-agreement: panels 61 · rr 0.67 · rg 0.64 · fg 0.57 · unanimous 0.44 · ground-loo 0.66 (n=41) · reproduce 34/27/0 · refute 26/35/0 · ground 40/21/0

## Confirmed findings

### Blocking

- [Blocking] §2.3 steps 2 and 4 — Tracking-to-cap (step 4) plus the integrator freeze while unselected (step 2) combine into a lock-up: the thermal candidate is reset to the applied cap every tick Shadow binds, so on a load jump it becomes the binding min() term at the old cap and the shadow's 10 W/s (300 MHz/s) ramp is wiped every tick; the cap rises only at the thermal PI's own rate, contradicting §2.3's "38 → 100 W in ~6 s" and §4 sim 4.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓) — unanimous Blocking
  evidence: tick-by-tick trace by all three seats against design.md:123-139 and :154-156: tick 1 thermal=38 (tracked+frozen), shadow=48 → cap=min(38,48)=38, selected=Thermal, shadow reset to 38; repeats. §4's own bars are mutually exclusive: unit test "after 300 s with the shadow binding, the thermal candidate is within one integral step of the cap" (design.md:315-316) vs sim 4 "cap ramps over ~6 s" (design.md:337). Ground seat grounded standard override practice (controlglobal.com "under the hood of override control"): the non-selected controller's output stays live via external reset feedback, never pinned + gated. Finding #33 (GPU-HOT recovery bounded by the PI rate) is the same mechanism.
  fix-shape hint: drop the "selected == Thermal on the previous PI tick" integrator gate (tracking alone is the anti-windup), or track the unselected candidate to cap only on hand-over rather than every tick, and let the shadow keep its own accumulated state while it is the min; re-run sim 4 against the unit bar.

- [Blocking] §2.4 Curve entry — The Curve state drops the prior design's debounced "argmax controllable and T* feasible" gate: when an uncontrollable sensor (ambient/charger) is the EC argmax — 11–16 % of loaded field samples — the fan is set by heat neither loop can touch, T* is inflated, both loops sit at err=0 or ratchet caps against a target that cannot move the fan, with no state change and no flag.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓) — unanimous Blocking
  evidence: design.md:176-181 Curve entry = view fresh + tread resolves + replica reconciled, no argmax term; "argmax controllable" survives only in §2.6's settle gate (grep); T*_floor clamp (design.md:192) is Held-only; DeviceUnreachable fires only at max/floor. Field counts recomputed by two seats: run-1789139478 220/1921 and run-1789067819 469/2906 loaded samples with ec_argmax ∈ {ambient, charger}. Prior spec §2.5 line 525 / fwloop.10 line 957 had the gate for the observed NVMe case. Contradicts Goal line 11 "The fan target is met by regulating each device's own EC temperature group".
  fix-shape hint: re-home the debounced argmax-controllable check as a Curve→Held (or a new "Uncontrollable" hold) transition in TStarSource, with a flag; add a sim leg with an ambient-dominated argmax.

- [Blocking] §2.6 step 1–3 / §2.5 step 3 / §2.7 — During calibration both loops are frozen for up to ~22 min while the step deliberately raises the GPU lock +500 MHz, but the GPU HOT guard's only actuation (ratcheting the frozen loop's `max`) changes nothing applied, on_calib_sample never runs the guard path, and step.rs's `EC_MAX_ABORT_C = 95` abort is dropped from the §2.6 rewrite — leaving only the latched GPU_TRIP_C 91 release during the one window the design intentionally raises GPU power.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓) — unanimous Blocking
  evidence: src/calib/step.rs:59-61 `EC_MAX_ABORT_C` + test `ec_over_95c_aborts_mid_step_and_restores_the_floor` (line 782); prior spec §3.3 line 767 / fwloop.13 state the 95 °C abort as a requirement; new §2.6 (a full §3.3 rewrite, not a carry-over) never mentions abort (grep: zero hits in design, tree, seed); eb9.8 acceptance lists no over-temp gate; controller.rs ~1291 dispatches Calibrating to on_calib_sample which never reaches the guard; settle cap 300→600 s, fit window 300→360 s, run twice.
  fix-shape hint: keep the EC-max abort (restore floor, skip step with a Noted reason) in the per-device step runner and state in §2.6 that GPU HOT / EC-max abort the step; give the guard precedence over the step writer.

### Should-fix

- [Should-fix] §2.5 bumpless entry — "first applied cap equals the running state plus headroom" is arithmetically wrong: min(max(floor,draw), draw+headroom) = draw, so entry caps at one instantaneous sample; an idle dGPU below gpu_floor_mhz gets both candidates clamped to the 1000 MHz floor on tick one.
  verdict: confirmed (reproduce ✓ / refute ✗-survived (dissent: REJECT) / ground ✓)
  evidence: design.md:136 cap=min; :234-236 seeding; eb9.3 acceptance "after seed(x, y) the first tick applies x (thermal)" encodes cap=draw; src/config.rs:100 gpu_floor_mhz 1000, shadow clamped to [floor,max] (design.md:135). Severity Should-fix not Blocking: refute seat is right that a GPU lock is a ceiling (Decisions item 4) so the idle-GPU case is not a forced clock, and ground seat shows the pinned+cool ramp recovers in seconds.
  fix-shape hint: correct the §2.5 sentence to state cap=max(floor, draw) on entry, seed from a short-window draw (not one sample), and let eb9.3's acceptance say so.

- [Should-fix] §2.3 step 3 / `shadow_band_c` — When pinned, the else branch (draw+headroom) and the pinned-and-cool branch (applied+headroom) differ by at most pin_margin (2 W of 10 W; 105 of 300 MHz), so the shadow ratchets essentially the same amount whether or not the group is inside the band; the band guard and its Config key are near-inert and untested.
  verdict: confirmed (reproduce ✓ / refute ✗-survived (dissent: REJECT) / ground ✓)
  evidence: design.md:129-135 step 3 text + pinned definition; §4 line 315-316 states "shadow rises one headroom per sample only while pinned and cool" — the "only" is not implemented by the algebra; eb9.4 acceptance tests only the pinned-and-cool ramp.
  fix-shape hint: make the else branch hold the shadow at the applied cap (no rise) when pinned but not cool, or delete shadow_band_c; add a pinned-not-cool unit test.

- [Should-fix] §2.3 step 3 vs step 5 — The GPU shadow's 300 MHz/sample rise is capped by the 105 MHz/s actuator rate limit applied in the same tick, so `shadow_headroom` GPU is inert as a rise-rate knob; recovering 1600 MHz takes ~15 s (the spec's own ≳15 s starvation bar) and §4 sim 4's "~6 s" GPU ramp is unsatisfiable above ~630 MHz.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓) — unanimous
  evidence: src/control/gpu_pid.rs:53 `RATE_LIMIT_MHZ = 105` per 1 Hz tick, carried into DeviceLoop per §3; design.md:139-140 step 5; :154-156 "≳15 s is the onset starvation"; :337 sim 4; config.rs:100 floor 1000 / ceiling 3090.
  fix-shape hint: state the GPU rise as rate-limited (headroom ≤ 105 or a larger GPU slew for rising shadow), and restate sim 4's GPU bar in MHz consistent with 105 MHz/s.

- [Should-fix] §2.3 defaults vs Gains — The shadow raises the cap 2–3 orders of magnitude faster than the thermal PI retracts it (CPU ~0.03 W/s at 5 °C error, GPU ~0.34 MHz/s) across a 50–90 s dead time, converting the load-jump fan overshoot into a slow post-jump temperature overshoot; no §4 test bounds group temperature above T* or settling after the ramp.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓) — unanimous
  evidence: Kc·e/Ti retraction recomputed by all seats (finding's GPU number was off by 5×; corrected 0.34 MHz/s does not change the conclusion); tracking (design.md:137-139) hands the PI the shadow's aggressive value; §4 sim 4 bounds only fan crest; Goal line 15 promises "a ramp of seconds rather than a fan overshoot".
  fix-shape hint: add a group-temperature overshoot / settling bar to sim 4, and consider a faster retraction path (e.g. shadow fall on err<0) or a lower headroom.

- [Should-fix] §2.3 Gains / step 2 / §4 P_full — Above ~2143 MHz the P_full table is flat (card power limit), a ~950 MHz zero-gain region inside [floor, gpu_max_mhz=3090] that the sat_dir clamp rule cannot see; an Auto entry seeded at a high boost clock walks back through the dead band at ~0.34 MHz/s (~45 min) before the lock does anything.
  verdict: confirmed (reproduce ✓ / refute ✗-survived (dissent: REJECT) / ground ✓)
  evidence: design.md:325-328 "extended flat to 3090 at 100 W"; :126-128 sat_dir keys on the output clamp only; :286 gpu_max_mhz sanitised to [floor, 3090]; ground seat grounded power-limit clipping (xda-developers GPU boost/power-limit article). Should-fix not Blocking: refute seat shows organic boost settles near the 2143 knee and the 88 °C ratchet + card's own loop backstop it (profile: hardware backstops remain).
  fix-shape hint: cap gpu_max_mhz's effective ceiling at the power-limit knee (from the LUT sweep or a measured value), or model the plateau in the sim so it can be seen.

- [Should-fix] §2.3 Gains GPU K ≈ 0.04 °C/MHz — Derived by multiplying the W/MHz slope by 0.8 °C/W, a CPU-plant thermal resistance; the spec's own gpu-burn fact implies ≈0.4 °C/W for the GPU path, so the default GPU Kc is ~2× too small; §4 reuses 0.8 for the simulated GPU node so sims validate the assumption against itself, and defaults now ship unconditionally.
  verdict: confirmed (reproduce ✗ / refute ✗-survived / ground ✓)
  evidence: design.md:161-163 and :331; prior spec ThermalPlant K 0.8 °C/W (line 851, CPU-dominated); idle die 42 °C → 82.4 °C under 100 W (prior spec Facts) ⇒ ≈0.40 °C/W; "Auto no longer requires calibration" (design.md:230, 267). Spike in the finding (two-level lock step under gpu-burn) is the cheap check.
  fix-shape hint: derive a GPU-specific °C/W from the gpu-burn data (or the §2.6 step run once by hand) and use it for both the default Kc and the §4 plant.

- [Should-fix] §2.6 steps 2–3 / §4 coupling — With STEP_W=15 W × 0.8 °C/W ≈ 12 °C on the CPU group and 0.1 °C/°C coupling, the cross term on the GPU group is ≈1.2 °C > the 1 °C rejection bar (GPU step ≈2.4 °C on the CPU group), so on the spec's own model both fits are rejected every run and both devices stay on defaults.
  verdict: confirmed (reproduce ✗ / refute ✗-survived / ground ✓)
  evidence: design.md:254-260 step sizes and rejection rule; :324-331 coupling and K; FIT_WINDOW_S=360 s lets the coupled response reach the bar at ~113 s; seed item 8 says the cross term should be "small"; eb9.8 requires "both fits" and "cross-term rejection" on one plant. demoted from the seats' Blocking: profile — calibration is optional, the fallback is safe defaults that are logged ("says so"), and the real-hardware coupling is unmeasured (finding #45 rejected), so the worst case is ~22 minutes wasted, not a control failure.
  fix-shape hint: set the cross-term threshold from the modelled coupling × expected primary response (e.g. 0.2 × ΔT_primary), or subtract the modelled cross term before comparing.

- [Should-fix] §2.3 step 6 / §2.5 step 5 / §2.7 — `Hold` has no actuator-mismatch or calibration freeze, and nothing feeds the actuator verification result back into DeviceLoop; the carried-over §2.9 rule "Mismatch ⇒ integrator Freeze(ActuatorMismatch) … first Verified unfreezes" has no mechanism once budget.rs's `Freeze` enum is deleted, and no bead owns it.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓) — unanimous
  evidence: src/control/budget.rs:192-215 `Freeze` + `step(err, freeze)`; §3 deletes budget.rs wholesale but lists only `Freeze::DemandLimited`; design.md:115 tick inputs have no freeze/verdict; prior spec lines 686-694 carried over via §2.7 "unchanged"; eb9.3 tick(input) signature and eb9.7 acceptance cover only the calibration freeze and the Mismatch re-write, not the per-axis integrator hold.
  fix-shape hint: add a `hold: ActuatorMismatch` input (or Hold variant) to DeviceLoop::tick that freezes both candidates and skips tracking until Verified, and give it to eb9.7.

- [Should-fix] §2.1 group max — Each group is an unfiltered max over raw positive readings; a single stuck-high sensor drives that device to its floor for the whole session with only an informational flag, and EC-MISMATCH (argmax-only reconciliation) cannot see a bad reading inside a non-argmax group.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓) — unanimous
  evidence: src/sensors/ec.rs:98 drops only `c <= 0.0`; design.md:96-99, :106-109, :200-203; the hot watchdog keys on `gpu_temp_c` (NVML die), not the EC group (watchdog.rs:93); prior spec documents fw-fanctrl's injected 50 °C quirk; §4 has no stuck-sensor test. Failure direction is safe (drive to floor), hence Should-fix.
  fix-shape hint: add a per-group plausibility bound (absolute ceiling and/or max jump per sample) in EcReading with a flag, and a stuck-sensor sim leg.

- [Should-fix] §4 Simulation — The sims validate the loops against exactly the constants the gains were derived from and drop the prior design's ±50 % plant perturbation runs and the no-relay/hunting criterion; a loop oscillating slowly within ±140 RPM passes every stated bar.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓) — unanimous
  evidence: design.md:158-165 vs :324-332 (same τ/K/θ/coupling); grep for relay/hunt/perturb/±50 over design and tree: zero hits; prior spec fwloop.17 line 966 required both; eb9.11/eb9.14 bars are "±150 RPM ≥ 90 % of a 30-min window".
  fix-shape hint: re-add the ±50 % K/τ/θ perturbation runs and the period-agnostic no-relay rule to eb9.11/eb9.14 acceptance.

- [Should-fix] §2.4 Held tuning — The RPM PI's θ_eff=90 s is the inner loop's sensor dead time, not the inner closed loop's response (λ=270 s); λ+θ_eff=1090 clears the spec's own ≥3×360 s rule by 10 s, the cascade separation sits at ~3× (literature's interaction zone), no margin analysis exists, and sim 6's 600 s bar is shorter than λ so a passing sim cannot validate the tuning.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✗)
  evidence: design.md:159 and :188-192; controlglobal.com cascade-control tips ("a lower loop must be 4 times faster… at ~3:1 peak and integrated error suffer"); ground seat's SIMC source (opticontrols cascade_control_perspective.pdf) shows IMC/λ tuning is stable at 2.2:1 — accepted as reducing risk, not as removing the missing-margin gap; reproduce seat concedes ma_interval is already folded into θ_eff.
  fix-shape hint: derive the Held λ from the inner closed-loop constant (λ_inner+θ) with an explicit ≥4–5× separation, and give sim 6 a cap-trace hunting check at the ~1000 s period.

- [Should-fix] §2.4 Held anti-windup — The rule holds the RPM PI only when both device loops report `Hold::Clamp`, not the common light-load case where both are `Hold::Shadow` (raising T* cannot raise draw, temperature or RPM), so T* winds to gpu_hot_c−2; no directional conditional integration at the RPM PI's own clamp either.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓) — unanimous
  evidence: design.md:142-143 Hold variants; :192-194 hold rule; :136 min-selector; eb9.16 reproduces the Clamp-only wording; prior spike fwloop.24 measured a 14 °C-equivalent overshoot from exactly this idle-windup-then-load pattern.
  fix-shape hint: hold the RPM PI when both loops are in {Clamp, Shadow, GroupUnavailable} (anything where the extra would not move), and apply sat_dir at its clamp.

- [Should-fix] §2.4 Held anti-windup — The "both report Hold::Clamp" rule can never fire when one device reports `GroupUnavailable` (dGPU unpowered — a normal, tested configuration), so with the CPU alone hard against max/floor the RPM PI winds T* to its clamp.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓) — unanimous
  evidence: design.md:97 "dGPU unpowered → GPU group None"; :121-122 early return marks GroupUnavailable before Clamp can be set; :192-195 rule; eb9.7 acceptance "GPU whose group reads None from the first tick"; eb9.16 restates the rule.
  fix-shape hint: treat GroupUnavailable as vacuously "no more room" in the RPM PI hold predicate.

- [Should-fix] §2.4 Held vs §2.3 step 6 — The T* hold predicate is stated in terms of `Hold::Clamp` "at max / at floor", but `Clamp` carries no bound identity and eb9.16 consumes only `Hold`; `Selected` reads Shadow when the thermal candidate is clamped at max but shadow wins the min, so the composite predicate is not implementable from the declared inputs.
  verdict: confirmed (reproduce ✓ / refute ✗-survived (dissent: REJECT) / ground ✓)
  evidence: design.md:142-143, :193, :136; task-tree-settled.md:159-161 eb9.16 consumes "both loops' Hold (eb9.3)"; refute seat's point that Selected exposes Floor/Max is answered by the ground seat's Selected::Shadow-with-thermal-at-max case.
  fix-shape hint: give `Clamp` a `Bound {Floor, Max}` payload (or pass thermal's own clamp state) and add it to eb9.16's consumes line.

- [Should-fix] §2.4 / §2.7 guards — A single T* feeds both device groups and may reach 86 °C (Held) or a 95 °C tread (Curve, quiet16 fixture), yet the design names no CPU-side ceiling or proportional guard: the only CPU protection is the latched 95 °C Tctl trip, which releases to stock (raising the fast limit 53 → 65–71 W).
  verdict: confirmed (reproduce ✓ / refute ✗-survived (dissent: REJECT) / ground ✓)
  evidence: design.md:191 Held clamp from the GPU guard only; §2.7 guard list has no CPU ratchet; src/control/watchdog.rs:17 `CPU_TRIP_C = 95.0`; tests/fixtures/fanctrl/print_all_quiet16.json has a tread at 95 °C/100 %; Facts: stock fast limit 65–71 W vs the daemon's 53 W. Refute seat's `cpu_max_w` clamp is a power ceiling, not a temperature guard.
  fix-shape hint: add a CPU hot guard (e.g. Tctl ≥ 90 ratchets cpu max down) mirroring the GPU MaxRatchet, and clamp Curve-state T* to min(tread, cpu_hot_c−margin, gpu_hot_c−2).

- [Should-fix] §2.5 step 3 GPU HOT ratchet — Recovery is symmetric and unconditioned on temperature, so once the guard clears at 86 °C `max` climbs straight back at the same rate until it re-trips at 88 °C: a limit cycle the design neither bounds nor tests; the prior design's "5 min GPU HOT episode, no post-episode overshoot > 150 RPM" bar was dropped.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓) — unanimous
  evidence: design.md:217-221; guard keys on NVML die temp, distinct from the EC group the PI regulates (task-tree-settled.md:99, :141); eb9.15 tests the ratchet helper in isolation; eb9.14 only requires the Floor leg be reached; prior spec line 1046.
  fix-shape hint: gate recovery on die temperature margin (e.g. resume rising only below exit−N °C, or at a slower rate), and add a closed-loop GPU HOT episode sim with an RPM/clock overshoot bar.

- [Should-fix] §2.5 step 3 + §2.3 step 4 — Recovery from a GPU HOT trip is bounded by the thermal integrator's rate, not the 105 MHz/s ratchet: once max has ratcheted to the floor the tracked thermal candidate sits at the floor, becomes the min on the first recovery tick, and climbs at ~0.35–1.4 MHz/s (tens of minutes); no recovery-time bar exists.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✗)
  evidence: same lock-up mechanism as the first Blocking finding: tracking sets thermal := cap each tick while Max binds, so on the tick max steps up 105 MHz the tracked thermal (old max) is lower and gets selected; ground seat's rebuttal assumes tracking carries thermal up with max and misses the hand-over tick. Prior spec fwloop.17 line 966 had a post-episode bound; eb9.14/eb9.15 have none.
  fix-shape hint: fixing the tracking/freeze lock-up resolves this; add a post-GPU-HOT recovery-time bar to eb9.14.

- [Should-fix] Decisions 4 / §2.3 step 3 / §4 — The GPU shadow cap's justification ("same work at a lower V/F point… pays for itself in watt-hours") is unmeasured, is contradicted by §4's own model (P_full is flat above 2143 MHz, so the boosting case draws less in the model), is inert while power-limited, and the pinned-and-cool ratchet drives a pinned GPU's shadow to max within seconds; sims assert no energy metric and telemetry has no frame-time field.
  verdict: confirmed (reproduce ✓ / refute ✗-survived (dissent: REJECT) / ground ✓)
  evidence: two seats independently replayed run-1789067819 (median SM clock 1192 MHz at ≥100 % util; power plateau ~100 W at 2092–2130 MHz) and run-1789139478 (median 1335 MHz at 60–79 %); design.md:26-31, :121-127, :149, :325; ground seat's NVIDIA-forum citation did not resolve but is not load-bearing.
  fix-shape hint: run the finding's spike (offline replay + 10 min shadow-on/off during the gaming check) before eb9.4's GPU defaults ship; if it does not bind or save, default the GPU headroom much higher or make the GPU shadow opt-in.

- [Should-fix] §3 Deletions — Deleting `src/control/allocator.rs` wholesale orphans `CPU_MAX_W` (the default and sanitiser bound of the retained `Config::cpu_max_w`), and no bead owns rehoming it, so eb9.12's own "cargo test + clippy green" acceptance cannot hold. (The `DOWN_RATE_W` half of the claim is refuted: eb9.15 replaces `gpu_share_override`, its only consumer, before the sweep.)
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✗ — ground rejected only the DOWN_RATE_W half)
  evidence: src/config.rs:12 `use crate::control::allocator::{CPU_MAX_W, GPU_MAX_W}`, :102, :202-208; allocator.rs:44 sole definition; §2.8 keeps cpu_max_w; eb9.6/eb9.12 Files lists exclude the move; eb9.17 runs after eb9.12.
  fix-shape hint: move `CPU_MAX_W`/`CPU_MAX_W_FLOOR` into config.rs in eb9.6 (before the sweep) and say so in eb9.12.

- [Should-fix] §2.6 / §2.8 fitted gains — Fitted gains are one unkeyed pair per device although the step test runs with fw-fanctrl/EC fan regulation live, so K and τ embed the fan's closed-loop rejection at one strategy/duty/AC state and are reused across quiet16/cool16 and every fan target — unlike warm start, keyed by (strategy, duty, on_ac) — with no invalidation rule.
  verdict: confirmed (reproduce ✓ / refute ✗-survived (dissent: REJECT) / ground ✓)
  evidence: design.md:246-262 freezes only the device loops; :279-280 vs :228; prior spec line 859 ran the step "once per active strategy" and line 347-353 documents a ~4× plant-gain difference between quiet16 and cool16 treads; refute seat's reliance on ±50 % perturbation runs is contradicted by the confirmed §4 finding above (those runs were dropped).
  fix-shape hint: key `cpu_gains`/`gpu_gains` by strategy (or at least invalidate on strategy / ma_interval change), or freeze the fan (static duty) during the step.

- [Should-fix] §2.3 step 1 `GroupUnavailable` — No time bound, escalation, flag or release rule: a device whose group stops reading while the rest of the EC stays valid (two-label fault the whole-EC watchdog does not catch) is capped at its last applied value (or max/cpu_max_w) indefinitely with no thermal feedback from the loop; only the separate 95 °C Tctl / 91 °C NVML trips remain.
  verdict: confirmed (reproduce ✓ / refute ✗-survived (dissent: REJECT) / ground ✓)
  evidence: design.md:121-122, :196 Released triggers, :201 DeviceUnreachable needs group_c; src/control/watchdog.rs:28-90 trips on `cpu_temp_valid` (Tctl), independent of the cros_ec group labels; eb9.3 acceptance holds the cap with no timeout; eb9.10 fault injection covers only dGPU power-off.
  fix-shape hint: after a bounded GroupUnavailable dwell (e.g. 60 s) fall to Released for that device (or to floor) and raise a flag; add a CPU-group-dropout fault-injection leg.

- [Should-fix] §2.3 inputs / §2.5 — `draw` has no validity gate but the sampler flattens an unavailable RAPL/NVML reading to 0.0 (first sample and every post-resume counter reset), so one missing reading sets shadow = 0+headroom and slams the cap to its floor, and engaging Auto on such a sample seeds both candidates at the floor — the bump §2.5 says is impossible by construction.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓) — unanimous
  evidence: src/sensors/sampler.rs:206-215 `.unwrap_or(0.0)` with the "no validity flag" comment; src/sensors/rapl.rs:51-67 returns None on first call and on plausibility-rejected resume deltas; both field runs start with `cpu_pkg_w: 0.0`; allocator.rs:98 already gates on `gpu_mhz_valid`, which §2.3 drops; prior spec line 361-363 fixed the same floor-slam class.
  fix-shape hint: make `draw` an `Option` (or gate on a validity flag) and hold the last cap / skip seeding when it is absent.

- [Should-fix] §2.4 Curve re-derivation — T* re-derivation and `resync_error` are specified to fire on every `view_changed` rather than on a change in the curve points as mode.rs does today, which on field cadence (view_changed ≈ 1 sample in 30) zeroes both loops' proportional term roughly every sixth PI tick; the spec claims parity with the prior rule and no bead owns the points-keyed narrowing.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓) — unanimous
  evidence: src/control/mode.rs:370-381 `points_changed = view.curve != self.cached_points`; prior spec fwloop.10 line 957; design.md §2.4 "on every view_changed… exactly as §2.4 required"; view_changed rate recomputed: 828/24822 and 100/2982 samples; eb9.5 restates the loose wording with no points-cache test.
  fix-shape hint: keep the points-keyed cache (re-derive only when curve points or target_duty change) and add the negative test to eb9.5.

- [Should-fix] §2.3 step 2 / §2.5 steps 4–5 / §2.7 — The cadence of actuator writes is never stated now that the cap can change every sample: the falling shadow crosses the 0.5 W grid every ~1.5 s and the rising one every second, so the change-gated synchronous write (ryzenadj + read-back, NVML + verify_lock, RUN_TIMEOUT 5 s each, up to five commands) now fires ~3–5× today's rate on an unbounded channel drained one sample at a time, with no deadband, rate cap or per-sample budget.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓) — unanimous
  evidence: src/control/controller.rs:2040-2042 change gate, :1562 ALLOC_PERIOD_S gate today, :2896-2911 one sample per select!; src/actuators/cmd.rs:35 RUN_TIMEOUT; src/main.rs:32/:352-361 unbounded channel and five-command worst case; grep for deadband/backlog/cadence in design + tree: zero hits.
  fix-shape hint: state a write deadband (only write on a grid change ≥ N) or a minimum write interval, and note that the PI/shadow math is t_mono-driven so a backlog lags the hardware rather than the maths.

- [Should-fix] §2.7 `verify_lock` "unchanged" — `GpuLockVerifier` is scoped to one locked value and needs three consecutive strikes; the controller rebuilds it on any change of the applied clock, so while the shadow or the GPU HOT ratchet moves the lock every sample (the design's own steady-state and safety cases) the streak resets every tick and stickiness detection can never trip.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓) — unanimous
  evidence: src/actuators/gpu.rs:49-93 `VERIFY_STRIKES = 3`, one state machine per locked value; src/control/controller.rs:2140-2153 `if auto.gpu_verifier_mhz != Some(locked) { GpuLockVerifier::new(locked) }`; design.md:129-135, :217-221; prior spec fwloop.12 line 960 relies on the 3-strike Mismatch.
  fix-shape hint: verify against "reported ≤ commanded + slack" with a streak that survives lock changes (or a tolerance window), instead of resetting per value.

- [Should-fix] §4 GPU simulation model — "the reported SM clock is the lock when load_level ≥ 0.9" contradicts the same paragraph's P_full plateau at 100 W (~2143 MHz) and the repo's own field case (lock 3090, card at ~2520 MHz), so the GPU pinned test is TRUE in simulation and FALSE on hardware above the knee; sims 1, 2, 4, 5 validate an optimistic pinned test and the parenthetical "so the pinned test behaves as on hardware" asserts the opposite.
  verdict: confirmed (reproduce ✓ / refute ✗-survived (dissent: REJECT) / ground ✓)
  evidence: design.md:325-332; src/calib/lut_sweep.rs:426-436 `power_limited_gpu_pins_below_the_lock`; the design's symmetric 105 MHz pin_margin vs lut_sweep's one-sided `actual ≤ lock + slack`.
  fix-shape hint: model reported clock = min(lock, clock_at_power_limit(load)) in the plant, and decide explicitly whether a power-limited card counts as pinned (one-sided test) — this also settles the rejected pinned-test finding below.

- [Should-fix] §2.3 defaults table `pin_margin` (promoted from spot) — Both margins are presented as existing constants but the values do not match the code (`CPU_PINNED_MARGIN_W` is 1.5 W, not 2 W; `GPU_PINNED_MARGIN_MHZ` is 30 MHz, not 105 MHz — a 3.5× widening the spec never argues for, likely conflated with the 105 MHz/s rate limit), and eb9.3 copies the wrong numbers as "moved from allocator::demand".
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓) — unanimous (severities SF/SF/Nit)
  evidence: src/control/allocator.rs:50, :53, :56; design.md:152; task-tree-settled.md:25.
  fix-shape hint: either state the new values as deliberate changes with a reason, or restore 1.5 W / 30 MHz; fix eb9.3's text.

### Nit

- [Nit] §2.3 defaults preamble — The shadow Config keys get defaults but no sanitiser ranges; `shadow_headroom = 0` pins the cap at the entry draw for the session and `shadow_fall_rate = 0` never lets the shadow fall.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓) — unanimous (seats: Should-fix)
  evidence: design.md:145-152 vs :286 (explicit range only for gpu_max_mhz); src/config.rs:229 existing sanitiser admits 0 for a floor key; eb9.6 acceptance "sanitizer tests per new key" with no stated bounds.
  demoted: profile states a single operator on one machine; both failures require the operator to write a 0 into their own config, and rollback is a config edit + restart.
  fix-shape hint: give both keys a positive floor in the sanitiser (e.g. ≥ 1 W / ≥ 30 MHz, ≥ 0.05 W/s / ≥ 1 MHz/s) and say so in §2.8.

- [Nit] §2.8 / eb9.7 `t_star_last_good` — The spec never says when the field is written; eb9.7 says "on change", which in Held means a synchronous state.json rewrite (no .bak) every 5 s PI tick for the duration of a curve outage, while a shutdown-only write loses the value on an unclean exit — the case the field exists for; controller.rs's own rule is "never per-update (no disk churn)".
  verdict: confirmed (reproduce ✗ / refute ✗-survived / ground ✓)
  evidence: design.md:184-191, §2.8 "No .bak"; task-tree-settled.md:59-60; src/control/controller.rs:2690-2692; src/state.rs:140.
  demoted: profile states resilience polish is low-value and rollback is trivial; the worst case is SSD churn or a T* seed falling back to the 75 °C default on an unclean exit.
  fix-shape hint: write on Held exit / Auto exit / a dirty-flag interval (e.g. 60 s), not per change.

## Not verified (beyond panel cap)
- none

## Beyond remainder cap (count only)
- none

## Rejected (with reason)
- §2.3 step 3 pinned-and-cool starvation fixed point (suggested Blocking) — reproduce and ground REJECT: the else branch still yields draw+headroom ≈ applied+headroom−pin_margin, i.e. the shadow keeps growing (which is exactly why the band-guard finding above is confirmed), and the spec scopes the 6 s figure to "while the device stays pinned and cool" (design.md:154-155; seed lines 76-78). Refute seat's CONFIRM stands as a minority view.
- §2.3 step 3 GPU shadow algebraic self-feedback to ~430 MHz (suggested Blocking) — refute: the recursion ignores the 10 MHz/s fall rate; ground: the fixed point is shadow = draw + headroom, which is the Goal's stated steady state and §4 test 1's pass condition. The reproduce seat's reframing (sub-90 % util transitions converge slowly) is noted but was not the claim.
- §2.3 step 5 "as the allocator does today" ambiguity over UP_RATE_W/DOWN_RATE_W (suggested Blocking) — step 5's colon list is exhaustive (0.5 W grid, 105 MHz/s), eb9.3 names only those two, and allocator.rs is deleted wholesale in §3/eb9.12; the drop is stated, not silent.
- §2.3 GPU gain varies >3× across the clock range (suggested Should-fix) — worst-case destabilising mismatch vs nominal K is ~1.6×, inside the ≥2 gain-margin bar; the "gain schedule" quote refers to the RPM loop's slope schedule, not a GPU thermal schedule; the flat region is handled by the clamp/sat_dir rule.
- §2.6 step test has no load precondition (suggested Should-fix) — §2.6 explicitly names the no-response case ("a light load that never heats… rejected… the runner reports it") and step.rs already carries PIN_UTIL_MIN_PCT/needs_load nagging that §3 does not delete.
- §2.6 +500 MHz step lands in the power-limited flat region (suggested Should-fix) — covered by the same "a device whose group cannot respond… rejected by the magnitude rule" clause; running on defaults is the declared-safe fallback.
- §2.4 Curve-state T* unclamped / infeasible (suggested Blocking) — device outputs are still clamped to [floor, max] (design.md:117), the prior design also ended at floor+flag for this case, and the high side is bounded by the T*-independent GPU guard; the design names "the real one" case explicitly.
- §2.1 gpu_amb is an ambient sensor (suggested Should-fix) — two seats checked the field data: gpu_amb is argmax only during heavy GPU load (541/4863 samples in run-1789067819, 0/24182 in run-1789139478) and rises 51 → 64-69 °C under gpu-burn while `ambient_f75303@4d` moves ~3 °C; the premise is empirically false.
- §2.4 Held λ=1000 s cannot meet sim 6's 10-min bar above ~275 RPM initial error (suggested Should-fix) — sim 6 is a mid-session curve loss from a converged Curve state (initial error ≤150 RPM); the restart-from-stale-seed case the finding describes has no 10-min bar in §4.
- §2.4 Held: stalled/static fan winds T* to gpu_hot_c−2 (suggested Should-fix) — `TargetUnreachable (high)` ("when T* would exceed the guard ceiling") fires at exactly that saturation and is the restated successor of the prior (high) rule; the ceiling is 2 °C under an independent NVML guard.
- §2.4 Held T* ceiling mixes NVML die and EC group units (suggested Should-fix) — the offset is measured in the prior spec's Facts (gpu_vr mean 83.2 vs die 82.4 under the 100 W burn, "~1 °C above the die"), in the safe direction.
- §2.5 step 3 GPU HOT ratchet "per allocation tick" 5× ambiguity (suggested Should-fix) — the parenthetical ties 105 to "one actuator rate-limit step", i.e. §2.3 step 5's 105 MHz/s at the 1 Hz sample cadence; stale word, no live second cadence.
- §2.5 / §2.3 floor ≤ max invariant not stated (suggested Should-fix) — "to no lower than floor" is the invariant, and eb9.15's MaxRatchet is unit-tested for the floor stop; gpu_pid.rs is not deleted wholesale.
- §2.8 `with_lut_floor_clamp` orphaned (suggested Should-fix) — the cross-unit (W vs MHz) inversion it guarded no longer exists; "gpu_max_mhz sanitised to [gpu_floor_mhz, 3090]" is the same-unit replacement and eb9.6 owns it; the seed's deletion list names the function.
- §2.3 "38 → 100 W" example exceeds the 54 W CPU cap (suggested Should-fix) — `max` is a free DeviceLoop input and the crate's unit tests already pass arbitrary bounds (allocator.rs:391-394, sim_tests.rs), so eb9.4's acceptance does not force a production cpu_max_w change. The prose example is still sloppy — worth a one-line edit when the §2.3 numbers are reworked for the Blocking lock-up.
- §2.3 GroupUnavailable → available recovery unstated (suggested Should-fix) — the early return freezes both candidates and the output is clamped + slew-limited (105 MHz/s), so the worst recovery tick degrades to the same bounded ramp the spec accepts for load steps; a prose clarification at most.
- §4 0.1 °C/°C coupling unsourced (suggested Should-fix) — the §2.6 1 °C threshold is a real-hardware guard from the seed's field observation, not calibrated against the sim constant, and Decision 8 explicitly parks hardware validation; a spike suggestion, not a contract violation.
- §1 two SISO loops on a 2×2 plant with no detuning (suggested Should-fix) — the plants have distinct actuators (ryzenadj W vs NVML MHz), §4 models the coupling (0.1 °C/°C) and sim 3 (both heavy) is a common-mode bar; the ask is for a formal RGA/detuning derivation beyond the spec's stated sim-based bar.
- §2.3 step 2 integrator gate vs per-sample tracking cadence (suggested Should-fix) — "previous PI tick" names one sample and tracking's per-sample write is deterministic; the composition is intricate but not implementation-defined. (The interaction is in any case subsumed by the confirmed Blocking lock-up.)
- §2.3 step 4 tracking signal is pre-slew (suggested Should-fix) — Decisions 4 defines `applied` as the min of the candidates, §4's descent-from-max bar is phrased in terms of `cap`, and the refute seat's fetch of the cited controlglobal article shows external reset feedback uses the selected output, not the post-actuator value.
- §2.4 Released → re-entry shadow seeding unstated (suggested Should-fix) — §2.4 uses the `seed(thermal_at, shadow_at)` verb (not `resync_error`), whose signature requires a shadow value, and §2.3's instant-rise/rate-limited-fall dynamics self-heal both directions within a tick.
- §2.4 Released has no hysteresis (suggested Should-fix) — NVML failures are not a Released trigger; immediate hard-fault exit is the prior design's explicit, roasted decision carried over "as today", and re-entry seeds from live draw.
- §2.3 GPU pinned test false when power-limited (suggested Should-fix) — pin_margin deliberately reuses allocator::demand's "using its cap" semantics, not lut_sweep's one-sided lock-in-force test; the else branch tracks draw+headroom every tick, so there is no wait-state stall. (The sim-model half of this concern is confirmed above under §4.)
- §2.3 bursty CPU load fixed point below cap (suggested Should-fix) — this is the Goal's stated behaviour ("sits at a shadow cap just above its draw") and Decision 4's explicit CPU tradeoff ("the win is boost-peak clipping only"); clipped peaks are pinned samples by definition.
- eb9.1 replaces the cros_ec_dgpu_on fixture (suggested Should-fix) — docs/research/05-fw-fanctrl-loop.md re-measured the "−150" as an EC threshold sentinel, not a reading; the powered-under-load fixture matches the 2026-09-09 gpu-burn captures and the unpowered→None case is still tested.
- §2.3/§2.6 dead-time conventions inconsistent (suggested Should-fix) — the carried §3.3 states the reason twice: the fit runs on the already-filtered EC average so θ̂ contains the boxcar; adding ma_interval/2 again would halve Kc.
- §2.4 no escalation path when a device is unreachable in Curve (suggested Should-fix) — fw-fanctrl's own P-only curve responds to the real elevated EC reading (prior spec line 170-173), which is the escalation; the prior design also left unreachable targets as flag + floor hold.
- §2.9 v3 decision line omits `pinned` and `draw` (suggested Should-fix) — `err_c` (= T*−group_c) is on the decision line and `band` is a constant; `draw` stays on the unchanged sample line and joins by t_mono, exactly as the cited runs were already analysed.

## Unverified nits (spot-checked)
- [Nit] §2.4 `BOUND_HOLD` is used as the DeviceUnreachable dwell but never given a value, default or Config key (spot refute: CONFIRM — appears once in the spec, once in eb9.16, no number anywhere).
- [Nit] §4 has no T* step test (fan-target/strategy change in Curve), the one transition where the proportional kick is suppressed (spot refute: CONFIRM — only a unit-level resync_error assertion exists).
- [Nit] §2.8 adds eight Config keys and justifies only the gains pair; the five shadow tunables are hard-code-shaped like DOWN_RATE_MHZ/FIT_WINDOW_S (spot refute: CONFIRM).
- [FYI] No flag when both devices are shadow-capped and the fan misses target (spot refute: REJECT — `Hold::Shadow` is on the v3 line/TUI and is the Goal's normal light-load state).
- [FYI] Shadow uses raw draw with no deadband (spot refute: REJECT — the 0.5 W grid / 105 MHz/s slew bound the applied cap; field |Δdraw| mean 0.45 W).
- [FYI] Header carry-over list omits prior §2.7/§2.3 (spot refute: REJECT — §2.4 restates feasibility fully in the new vocabulary).
- [FYI] No periodic/bursty-load sim (spot refute: REJECT — Decision 8 parks realistic gaming load to the hardware check; shadow rise is not rate-limited on rising draw).
- [FYI] Held anti-windup reads an instantaneous Hold one sample out of phase (spot refute: REJECT — eb9.16 asserts the one-sample lag explicitly; step bound 0.5 °C/tick).
- [FYI] Rates in seconds, sampler best-effort with no dt (spot refute: REJECT — SAMPLE_PERIOD/SAMPLE_PERIOD_S coupling was fixed in roast PR-2/PR-3; sysfs jitter is ms).
- [FYI] θ_eff assumes a filled 60 s boxcar (spot refute: REJECT — a partially-filled boxcar has less lag, i.e. more margin).
- [FYI] Device loops fed the boxcar not the instantaneous reading (spot refute: REJECT — carried §2.2 architecture; gains are fitted against the filtered plant).
- [FYI] Two cadences in one DeviceLoop (spot refute: REJECT — design preference; "as today" is accurate).
- [FYI] GPU lock realised on a 15 MHz lattice, no quantisation rule (spot refute: REJECT — this card bins at ~7.5 MHz per gpu.rs's comment; the driver snaps; integrator is continuous in software).
- [FYI] §4 GPU power model has no static intercept (spot refute: REJECT — draw_w feeds only the sim's heat, not the shadow; sim 5 is heavy-load).
- [FYI] Held's third PI is YAGNI vs freezing T* (spot refute: REJECT — it is the prior design's roasted Mode B and Decision 6's content; a frozen T* cannot meet sim 6's RPM bar).
- [FYI] Calibration subsystem cost vs optional defaults (spot refute: REJECT — user-decided item 7; the 2026-09-10 failure was answered with parameter fixes).
- [FYI] Warm start retained despite bumpless entry (spot refute: REJECT — it also drives DutyRpmTable refinement and is a migration of existing code).
- [FYI] Hold and Selected overlap (spot refute: REJECT — deliberate denormalisation for telemetry/TUI).
- [FYI] Selected::Floor/Max have no consumer (spot refute: REJECT — §2.4's hold rule and DeviceUnreachable need the floor/max distinction).
- [FYI] SteepCurve retained with no consumer (spot refute: REJECT — explicitly informational, same boilerplate as every flag).
- [FYI] DeviceLoop generic over unit for two instantiations (spot refute: REJECT — shares nontrivial PI/shadow/selector logic between two live devices).
- [FYI] `max(90, 3·θ_eff)` floor never binds (spot refute: REJECT — general rule, fitted θ_eff can be small; inert not wrong).
- [FYI] RPM PI 0.5 °C step bound unreachable at the gains (spot refute: REJECT — defensive clamp; raw-fan fallback can trigger it; unit-testable directly).
- [FYI] VR/VRAM spike tests an in-group swap that cannot matter (spot refute: REJECT — the spec says so itself; user-parked spike).
- [FYI] VR/VRAM spike "before the group sets are frozen" vs eb9.1 unblocked (spot refute: REJECT — hardware-only, parked in run.md; eb9.1 carries the reasoning).

## Escalations (need human)
- §2.4 Held fixed plant gain 78 RPM/°C with the gain schedule deleted (EC-autofan curve ~0 RPM/°C at 67–73 °C, 140–420 RPM/°C below 64 °C) — material dissent not resolved: both REJECT seats rest on fwloop.16/fwloop.17 acceptance runs (EC-autofan authority run, no-hunt bar) that exist only in the superseded 2026-09-07 tree — a grep of task-tree-settled.md and the design doc finds no fwloop.16/17, no "autofan", no perturbation runs, and finding #20's unanimous panel confirms those runs were dropped — while the one CONFIRM seat's grep evidence stands; a human must decide whether Held needs a slope-scaled Kc / 0.25× floor or an EC-autofan sim leg before eb9.16 is implemented.
---

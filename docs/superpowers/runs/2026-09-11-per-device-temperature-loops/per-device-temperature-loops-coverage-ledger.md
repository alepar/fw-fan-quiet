# Coverage ledger — per-device-temperature-loops (root epic fw-fanctrl-loop-eb9)

One line per disposed finding: `id · type · disposition · one-line description`. Dispositions
are automatic (autonomous run); every applied fix names the bead(s) it changed.

## Round 1 (2026-09-11, 3 reviewers, 40 raw → 16 deduped)

- c1-01 · UNSATISFIABLE-ACCEPTANCE (unwired) · applied · eb9.7 seeds shadows but had no path to eb9.4 → edge eb9.7←eb9.4 + reason line.
- c1-02 · UNSATISFIABLE-ACCEPTANCE (unwired) · applied · eb9.13 consumes the v3 ControlStatus fields owned by eb9.9 → edge eb9.13←eb9.9.
- c1-03 · UNOWNED-SEAM · applied · decision-line emission call site: eb9.7 now owns it; edge eb9.7←eb9.9 (schema first).
- c1-04 · UNOWNED-SEAM · applied · DeviceLoop seed/resync API: eb9.3 owns seed()/resync_error() with no-kick acceptance; eb9.5/eb9.7 consume.
- c1-05 · UNOWNED-SEAM · applied · t_star_last_good load/persist wiring: eb9.7 owns it; edge eb9.5←eb9.6 for the field shape.
- c1-06 · GAP · applied · no fault injection for GroupUnavailable/Released: eb9.10 owns a fault-injection API (dGPU off, fan outage, EC invalid/stale view); eb9.14 cites it (needs: eb9.10).
- c1-07 · UNEXERCISED-CONFIGURATION · applied · Selected::Floor/Max never exercised: eb9.14's smoke now covers every Selected value (GPU-HOT-to-floor leg, unreachable at max).
- c1-08 · UNOWNED-SEAM · applied · sim_tests.rs edited by eb9.11 and eb9.12 unordered: eb9.11 owns removing the ClockWattsLut sims; eb9.12's list amended; edge eb9.12←eb9.11.
- c1-09 · UNSATISFIABLE-ACCEPTANCE (unwired) · applied · eb9.12's green gate did not wait for the acceptance sims → edges eb9.12←eb9.11, eb9.12←eb9.14.
- c1-10 · NARRATIVE-EDGE (unstated) · rejected (already applied before dispatch) · eb9.12←eb9.13 reason line was added between assembly and dispatch; the dump the reviewers saw predates it.
- c1-11 · UNOWNED-SEAM / GAP · applied · calibration freeze of both loops: eb9.7 owns freeze/unfreeze + reseed; edge eb9.8←eb9.7.
- c1-12 · UNSATISFIABLE-ACCEPTANCE (prose) · applied · eb9.3's "first-order plant" restated as a local test stub in device_loop.rs (no dependency on eb9.10).
- c1-13 · GAP · applied · per-device draw inputs plumbing: eb9.7 owns passing cpu_pkg_w / gpu_sm_mhz + gpu_util_pct into each tick.
- c1-14 · GAP · applied · duty↔RPM table refinement at warm start: eb9.7 owns it, unchanged from today.
- c1-15 · GAP · applied · WarmStart::key(strategy, duty, on_ac) form: eb9.6 owns it with a round-trip test.
- c1-16 · GAP · applied · telemetry schema docs at v3: eb9.9 owns the README/docs telemetry section.
- c1-17 · flag-sweep eb9.7 · applied (split) · the GPU HOT max ratchet rule is its own leaf eb9.15 (blocked by eb9.3, blocking eb9.7); the NOT CALIBRATED gate removal stays in eb9.7.
- c1-18 · flag-sweep eb9.5 · applied (split) · the Held driver (RPM PI), feasibility/steepness flags and DeviceUnreachable are their own leaf eb9.16 (blocked by eb9.5 and eb9.3); eb9.14's Held sim consumes it.
- c1-19 · flag-sweep eb9.8 · accepted · reason holds: one-to-one with §2.6, single file set.
- c1-20 · flag-sweep eb9.9 · accepted · reason holds: split to eb9.13 is visible; the two seams it left (c1-02, c1-03) are applied.
- c1-21 · flag-sweep eb9.11 · accepted · reason holds: split to eb9.14 is visible and contracted.
- c1-22 · UNSATISFIABLE-ACCEPTANCE (unwired) · applied · eb9.9's sample-line fields come from eb9.1 with no path → edge eb9.9←eb9.1 + reason line.
- R-new (accepted into the canonical list as R17, R18): R17 the calibration path freezes both loops at their applied caps for the run and reseeds them on exit with no cap step; R18 DeviceLoop exposes a seed/resync API the controller and TStarSource drive.

## Round 2 (2026-09-11, 3 reviewers, 14 raw → 13 deduped; all 13 are consequences of round-1 fixes or acceptance gaps; count shrank 22 → 13; no INSUFFICIENT-INPUT; not degraded)

- c2-01 · UNOWNED-SEAM/missing edge (from c1-18) · applied · flags moved to eb9.16 → edges eb9.9←eb9.16, eb9.13←eb9.16; eb9.16 owns the flag-value export.
- c2-02 · missing edge · applied · eb9.4←eb9.6 (shadow Config keys); eb9.4 owns the USE of the keys.
- c2-03 · missing edge (from c1-17) · applied · eb9.15←eb9.6 (gpu_max_mhz/gpu_floor_mhz).
- c2-04 · UNOWNED-SEAM · applied · TStarSource tick input plumbing + one-tick-lagged Hold: eb9.7 owns; edge eb9.7←eb9.16; eb9.16 acceptance asserts the lag.
- c2-05 · GAP (acceptance, R12) · applied · eb9.7 acceptance re-points guard/watchdog/fence/restore/emergency tests at the two loops.
- c2-06 · UNSATISFIABLE-ACCEPTANCE · applied · gpu_share_override added to eb9.12 deletion list; edge eb9.12←eb9.15.
- c2-07 · GAP (spec drift) · applied · warm-start fan-tolerance conjunct restored in eb9.7 + negative acceptance.
- c2-08 · GAP · applied · emergency-release path restored to eb9.7 rewiring list + acceptance.
- c2-09 · GAP/UNEXERCISED-CONFIGURATION · applied · GroupUnavailable at Auto entry: eb9.3 reports max with hold; eb9.7 skips the NVML write; eb9.14 smoke gains an entry-with-None leg.
- c2-10 · UNSATISFIABLE-ACCEPTANCE (compile break from c1-03) · applied · eb9.9 is additive-only (deprecated Budget* fields kept until eb9.12 removes them); acceptance restated.
- c2-11 · UNSATISFIABLE-ACCEPTANCE (r1 owns without acceptance) · applied · eb9.7 acceptance covers R17 freeze, t_star_last_good round trip, draw inputs, table refinement.
- c2-12 · UNSATISFIABLE-ACCEPTANCE (from c1-07) · applied · eb9.10 owns a scriptable GPU die temperature so the GPU-HOT-to-floor leg is producible; eb9.14 cites it.
- c2-13 · GAP (acceptance) · applied · eb9.4 acceptance covers shadow seeding and resync leaving the shadow untouched.
- Loop ended after round 2 (fixed cap). Integration sweep: eb9.17 depends on eb9.1..eb9.16.

## Design roast 1 dispositions (2026-09-11; report 2026-09-11-per-device-temperature-loops-roast-design-1.md; spec revision 2)

Every confirmed finding is design-level; each was applied as a spec rewrite (rev2 of §2.3/§2.4, amendments to §2.1/§2.5/§2.6/§2.7/§2.8/§3/§4/Decisions) plus bead description/acceptance amendments (marked "Roast d1 amendments" in the tree).
- d1-01 · Blocking · §2.3 tracking + freeze lock-up · applied · parking rule: thermal candidate parked (max, tracked) while err > band, active inside the band; no "integrate only when selected" gate → eb9.3, eb9.4.
- d1-02 · Blocking · §2.4 Curve lacks the controllable-argmax gate · applied · debounced controllable-argmax entry condition + new Uncontrollable state and flag → eb9.5, eb9.14.
- d1-03 · Blocking · §2.6 calibration without over-temp abort · applied · EC_MAX_ABORT_C 95 + GPU HOT / CPU hot aborts with cap restore → eb9.8.
- d1-04 · Should-fix · bumpless entry arithmetic · applied · SEED_WINDOW_N 5 mean draw, deferred while draw is None → eb9.3, eb9.7.
- d1-05 · Should-fix · shadow_band_c near-inert · applied · band is load-bearing (pinned-not-cool holds) → eb9.4.
- d1-06 · Should-fix · GPU shadow rise capped by the 105 MHz/s slew · applied · GPU_RISE_SLEW 300 MHz/s when shadow selected and rising → eb9.4, eb9.11 (sim 4 bar).
- d1-07 · Should-fix · post-jump temperature overshoot unbounded · applied · instant shadow fall when err < 0; sim 4 overshoot ≤ 2 °C bar → eb9.4, eb9.11.
- d1-08 · Should-fix · P_full dead band above 2143 MHz · applied · "power-limited counts as pinned" + plant model min(lock, clock_at_power_limit) → eb9.3, eb9.10.
- d1-09 · Should-fix · GPU K derived with 0.8 °C/W · applied · 0.4 °C/W, Kc ≈ 4.9 MHz/°C; sim plant 0.4 → spec Gains, eb9.10.
- d1-10 · Should-fix · cross-term rejection bar unreachable · applied · threshold max(1 °C, 0.2×ΔT_primary) → eb9.8.
- d1-11 · Should-fix · no actuator-mismatch hold · applied · ActuatorState input + Hold::ActuatorMismatch; Freeze enum deleted → eb9.3, eb9.7, eb9.12.
- d1-12 · Should-fix · stuck-high sensor in a group max · applied · plausibility filter + EcImplausible flag; sim 11 → eb9.1, eb9.10, eb9.14.
- d1-13 · Should-fix · sims lack perturbation / no-relay · applied · sim 10 (±50 %, period-agnostic no-relay) → eb9.11.
- d1-14 · Should-fix · Held λ too short vs cascade separation · applied · λ_held 1440 s, Kc ≈ 2.9e-4, sim 6 bar 20 min → eb9.16, eb9.14.
- d1-15 · Should-fix · Held anti-windup misses Hold::Shadow · applied · predicate over {Clamp, Parked, Shadow, GroupUnavailable, DrawUnavailable} + directional integration at the RPM PI's clamps → eb9.16.
- d1-16 · Should-fix · anti-windup never fires with GroupUnavailable · applied · same predicate (GroupUnavailable counts) → eb9.16.
- d1-17 · Should-fix · Clamp carries no bound · applied · Hold::Clamp(Bound{Floor,Max}) → eb9.3, eb9.16.
- d1-18 · Should-fix · no CPU-side ceiling · applied · cpu_hot_c 90 / exit 87 guard on Tctl, DOWN_RATE_W 2, T* clamp min(cpu_hot_c−2, gpu_hot_c−2) → eb9.15, eb9.7, eb9.6, eb9.5.
- d1-19 · Should-fix · GPU HOT recovery limit cycle · applied · temperature-gated recovery (die ≤ gpu_hot_c−4, half rate); sim 9 → eb9.15, eb9.7, eb9.14.
- d1-20 · Should-fix · GPU HOT recovery bounded by the integrator · applied · tracking during the ratchet + recovery bar in sim 9 → eb9.15, eb9.14.
- d1-21 · Should-fix · watt-hour claim unmeasured · applied · Decisions item 4 marks the claim unmeasured; gpu_shadow_enabled opt-out; no energy bar claimed → spec, eb9.4, eb9.6.
- d1-22 · Should-fix · CPU_MAX_W orphaned by the allocator deletion · applied · rehomed to config.rs by eb9.6 before the sweep → eb9.6, eb9.12.
- d1-23 · Should-fix · fitted gains unkeyed · applied · gains keyed by (strategy, ma_interval) → eb9.6, eb9.8.
- d1-24 · Should-fix · GroupUnavailable unbounded · applied · GROUP_UNAVAILABLE_DWELL_S 60 → release + GroupLost; sim leg → eb9.3, eb9.14.
- d1-25 · Should-fix · draw has no validity gate · applied · draw: Option, DrawUnavailable hold, seed deferred → eb9.3, eb9.7.
- d1-26 · Should-fix · re-derivation on every view_changed · applied · points-keyed re-derivation + negative test → eb9.5.
- d1-27 · Should-fix · write cadence unstated · applied · WRITE_MIN_INTERVAL_S 2 with floor/guard/Released/reassert exceptions → eb9.7.
- d1-28 · Should-fix · verify_lock resets on every lock change · applied · re-scoped to "reported ≤ commanded + slack" with a streak that survives lock changes → eb9.7 (src/actuators/gpu.rs).
- d1-29 · Should-fix · sim GPU model ignores the power plateau · applied · min(lock, clock_at_power_limit) → eb9.10.
- d1-30 · Should-fix · pin margins mis-stated · applied · existing constants 1.5 W / 30 MHz → spec, eb9.3.
- d1-31 · Nit · shadow keys have no sanitiser ranges · applied · floors (headroom ≥ 1 W / 30 MHz, fall ≥ 0.05 W/s / 1 MHz/s, band 0.5–10 °C) → eb9.6.
- d1-32 · Nit · t_star_last_good write cadence · applied · Held exit / Auto exit / ≤ every 60 s while dirty → eb9.5, eb9.7.
- Spot-checked nits (BOUND_HOLD value, T* step sim, shadow tunables) · applied · BOUND_HOLD_S 60 (eb9.16); the T* step leg folds into sim 10's no-relay window (eb9.11); the five shadow tunables stay Config keys (user-tunable acoustics knobs) — recorded, not changed.
- Escalation (Held plant gain vs the deleted slope schedule) · mitigated + parked · the slope schedule is restored for the RPM PI (Kc × slope_ref/max(slope,slope_ref) clamped [0.25, 1]; 0.25× with no resolvable slope) in eb9.16; whether an EC-autofan sim leg is also needed before eb9.16 lands is the user's call (run.md parked).

## Design roast 2 dispositions (2026-09-11; report 2026-09-11-per-device-temperature-loops-roast-design-2.md; spec revision 3)

Root cause of the Blocking cluster: rev2's parking rule + band. Rev3 replaces both with one rule — the thermal candidate is always live, never tracked while the device is cool (it saturates at max), and tracked one-sided (thermal := min(thermal, cap)) only while the group is above T*; the shadow is draw + headroom always (no pinned test), rises only while the group is at or below T*. Every bead body was rewritten wholesale (no amendment paragraphs remain).
- d2-01 · Blocking · hold-at-applied freezes the cap in-band · applied · band and parking deleted; hot-only one-sided tracking (§2.3 steps 2–4) → eb9.3, eb9.4.
- d2-02 · Blocking · parked candidate is a hysteresis-free relay · applied · no parking: the thermal state is never discarded; the cap moves at the rise slew or the PI's rate only; sim 10 gains a fluctuating-load leg → eb9.3, eb9.11.
- d2-03 · Blocking · sim 4 bar unsatisfiable / plateau walk from max · applied · the shadow tracks the reported clock so a power-limited card's cap sits one headroom above the knee; the crossing tracks the thermal candidate to that cap (≤ 300 MHz of dead zone); sim 4 bar re-set (≤ 4 °C cold / ≤ 2 °C warm, no GPU HOT trip, settle in 3 λ) with the dead-zone paragraph → spec §2.3, eb9.11.
- d2-04 · Blocking · Held anti-windup non-directional · applied · directional hold sets (up-blocking vs down-blocking; Shadow blocks up only, Clamp(Floor) down only; complement-of-None, exhaustive) → eb9.16.
- d2-05 · Blocking · jump rule ambiguous · applied · jump rule deleted; absolute stateless gate on every label; plausible-but-stuck fails safe and surfaces as DeviceUnreachable; sim 11 re-stated → eb9.1, eb9.14, eb9.16.
- d2-06 · Blocking · P_limit undefined · applied · no pinned test at all (draw + headroom always); P_limit gone → eb9.3, eb9.4, eb9.12.
- d2-07 · Blocking · bead bodies contradicted by amendments · applied · every leaf body rewritten wholesale against rev3 (eb9.1–eb9.16).
- d2-08 · Blocking · no state for Auto entry with an uncontrollable argmax / the hysteresis window · applied · Auto entry starts in Held (T* = last good or current controllable max); Held = "not curve-derived right now"; Uncontrollable entered from Held or Curve → eb9.5.
- d2-09 · Should-fix · Uncontrollable needs a tick input · applied · ThermalMode{Regulate, Bypass} on the tick contract (eb9.3), output by the T* source (eb9.5), plumbed by eb9.7.
- d2-10 · Should-fix · gpu_shadow_enabled off = no control · applied · with no parking, off = thermal-only GPU control (the A/B works); cost noted → spec, eb9.4.
- d2-11 · Should-fix · err<0 override contradiction · applied · rule collapsed: shadow rises only while err ≥ 0, never an instant drop → eb9.4.
- d2-12 · Should-fix · stuck at start · applied · accepted fail-safe disposition + DeviceUnreachable, sim 11 from-start leg → eb9.1, eb9.14.
- d2-13 · Should-fix · filter scoped to group max · applied · absolute gate applies before argmax/all; T*_floor uses plausible readings → eb9.1, eb9.16.
- d2-14 · Should-fix · GroupUnavailable dwell mis-scoped / action ambiguous · applied · absent-from-start vs lost-mid-session; release = cap max (not stock), written; sim 8 legs → eb9.3, eb9.7, eb9.14.
- d2-15 · Should-fix · eb9.6 Option<Gains> · applied · body rewritten: BTreeMap keyed map → eb9.6.
- d2-16 · Should-fix · gains precedence · applied · config > fitted > default, resolved by eb9.7, gains_source reported (eb9.9, eb9.13) → spec §2.3 Gains.
- d2-17 · Should-fix · hold set omits ActuatorMismatch · applied · complement-of-None directional sets → eb9.16.
- d2-18 · Should-fix · DrawUnavailable removes temperature regulation · applied · thermal regulation continues; shadow holds; 60 s dwell → thermal-only → eb9.3.
- d2-19 · Should-fix · T* clamp inversion · applied · T*_floor = min(raw floor, ceiling); TargetUnreachable(high) → eb9.16; cpu_hot_c sanitiser range → eb9.6.
- d2-20 · Should-fix · Released vs calibration · applied · calibration is a controller-level freeze, not a TStarSource state; "settled cap" defined → eb9.5, eb9.7, eb9.8.
- d2-21 · Should-fix · λ_held × 0.25 floor vs sim 6 bar · applied · sim 6 written against λ_eff with an EC-autofan leg (the d1 escalation's leg) and a quiet16 leg → eb9.14; escalation resolved (see run.md parked).
- d2-22 · Should-fix · §4 heat equation 0.8 · applied · 0.4 °C/W in the equation → spec §4, eb9.10.
- d2-23 · Should-fix · unit bar unsatisfiable · applied · unit bars rewritten for hot-only tracking → spec §4, eb9.3, eb9.4.
- d2-24 · Should-fix · CPU burst ratchets the shadow to max · applied · CPU draw = 5-sample tail mean; rise slew 10 W/s; burst example quantified → spec §2.3, eb9.7, eb9.10.
- d2-25 · Should-fix · CPU hot guard undebounced / no sim · applied · CPU_HOT_STREAK = 3; sim 9(b) → eb9.7, eb9.14, eb9.15.
- d2-26 · Should-fix · Mismatch freeze vs guard ratchet · applied · applied cap = min(cap, max) always; guards precede the freeze; immediate write → eb9.3, eb9.7.
- d2-27 · Nit · θ_eff frozen at 60 s · applied · default_gains(ma_interval) computed live → eb9.3, eb9.7.

## Design roast 3 dispositions (d3-01..d3-19; 2026-09-11; spec revision 4)

The authorized two-candidate alternative is retained with an edge-only measured thermal handover. The continuous draw-driven state overwrite is removed. All bead descriptions are rewritten wholesale. Offline arithmetic/trace evidence is documented; future implementation acceptance simulations are requirements, not claimed passes.

- d3-01 · Blocking · §2.3 hot draw dip tracking · applied · one-time measured handover; no continuous draw overwrite; both shadow directions hold while hot; recovery replay → eb9.3, eb9.4, eb9.11, eb9.14.
- d3-02 · Blocking · §2.2 group seed and view changes · applied · own instantaneous group seeds; view changes preserve history and set interval only → eb9.1, eb9.2, eb9.7.
- d3-03 · Blocking · §4 sim 4 dead-zone arithmetic · applied · separate downward-knee crossing bound from post-crossing delay and 3λ settling → eb9.3, eb9.10, eb9.11.
- d3-04 · Blocking · §2.5 verify_lock timing · applied · pair sample with successfully completed command; bounded predecessor allowance; compliant and ignoring-card vectors → eb9.7, eb9.14.
- d3-05 · Should-fix · §2.3 dt and resume · applied · explicit bounded elapsed time, PI accumulator, resume hold/reset and dwell semantics → eb9.2, eb9.3, eb9.7.
- d3-06 · Should-fix · §2.5 guard versus averaged group · applied · guard clamps PI independent of group error; both branches tested; recovery bars start at eligibility → eb9.3, eb9.7, eb9.14, eb9.15.
- d3-07 · Should-fix · §2.3/2.4 target-driven hot spell · applied · measured-only handover, ΔT* error-history shift, explicit Curve upward reset and recovery assertions → eb9.3, eb9.5, eb9.7, eb9.11, eb9.16.
- d3-08 · Should-fix · §2.8 last-good seed · applied · strategy/target/timestamp qualification, six-hour age, migration and restart cases → eb9.5, eb9.6, eb9.7, eb9.14, eb9.17.
- d3-09 · Should-fix · §2.4 Bypass transitions · applied · seed shadow on entry and thermal on exit from commanded cap; suppress transition motion → eb9.3, eb9.4, eb9.5, eb9.7, eb9.14.
- d3-10 · Should-fix · §2.1 plausibility/reconciliation · applied · positive-only raw reconciliation stream separate from plausible control data; no filter-induced mismatch sim → eb9.1, eb9.2, eb9.10, eb9.14.
- d3-11 · Should-fix · §2.4 DrawUnavailable anti-windup · applied · remove from both directional blocking sets; live thermal-only must-not-hold vector → eb9.3, eb9.16.
- d3-12 · Should-fix · §2.3 Bypass shadow gate · applied · Bypass ignores frozen T* for both shadow directions; hot-group load-rise case → eb9.3, eb9.4, eb9.14.
- d3-13 · Should-fix · §2.3 thermal-only plateau · applied · explicit full max-to-knee bound and disabled/missing-draw sim legs; slew on transitions → eb9.3, eb9.4, eb9.11, eb9.14.
- d3-14 · Should-fix · §2.3 bound tie identity · applied · Floor then Max precedence; interior Thermal ties; reachable clamp diagnostics and smoke → eb9.3, eb9.4, eb9.9, eb9.13, eb9.14, eb9.16.
- d3-15 · Should-fix · §2.3 GPU τ assumption · applied · offline group-fit evidence; provisional τ15/Ti15/Kc2.1 and crossed τ8–50 uncertainty grid → eb9.3, eb9.10, eb9.11.
- d3-16 · Should-fix · §2.4 stuck uncontrollable argmax · applied · rolling unchanged dwell, visible quarantine, Held/Regulate backstop and recovery → eb9.1, eb9.5, eb9.9, eb9.10, eb9.13, eb9.14, eb9.16.
- d3-17 · Should-fix · §2.1 unknown labels · applied · exact plus prefix fallback; unknown/missing label diagnostics; unrelated Unknown routes Held → eb9.1, eb9.5, eb9.9, eb9.13, eb9.14.
- d3-18 · Should-fix · §2.4 empty uncontrollable set · applied · finite collapsed interval plus explicit diagnostic and recovery tests → eb9.5, eb9.9, eb9.13, eb9.14, eb9.16.
- d3-19 · Should-fix · §2.8 CPU hot sanitizer · applied · independent [82,94] range and relative exit/recovery thresholds → eb9.6, eb9.7, eb9.14, eb9.15, eb9.16.

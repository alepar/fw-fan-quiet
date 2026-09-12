#!/usr/bin/env python3
"""Rewrite epic eb9 descriptions from the revision-4 design, then render the live tree."""

import json
import subprocess
import sys
from pathlib import Path


ROOT = "fw-fanctrl-loop-eb9"
IDS = [ROOT, *(f"{ROOT}.{n}" for n in range(1, 18))]
HERE = Path(__file__).resolve().parent
RUN_DIR = HERE.parent
SNAPSHOT = HERE / "beads-before-rev4.json"
TREE = RUN_DIR / "task-tree-settled.md"


def bd(*args: str) -> str:
    result = subprocess.run(["bd", *args], capture_output=True, text=True)
    if result.returncode:
        print("FAILED:", "bd", *args, file=sys.stderr)
        print(result.stdout, file=sys.stderr)
        print(result.stderr, file=sys.stderr)
        raise SystemExit(result.returncode)
    return result.stdout


def show(issue_id: str) -> dict:
    value = json.loads(bd("show", issue_id, "--json"))
    return value[0] if isinstance(value, list) else value


def issue_snapshot(issue: dict) -> dict:
    return {
        "id": issue["id"],
        "title": issue["title"],
        "description": issue["description"],
        "status": issue["status"],
        "priority": issue["priority"],
        "issue_type": issue["issue_type"],
        "parent": issue.get("parent"),
        "dependencies": [
            {"id": dep["id"], "dependency_type": dep.get("dependency_type", "blocks")}
            for dep in issue.get("dependencies", [])
        ],
    }


def graph_signature(issue: dict) -> dict:
    snap = issue_snapshot(issue)
    snap.pop("description")
    return snap


BODIES = {
    ROOT: """Root epic for the 2026-09-11 super-auto run (branch super-auto/per-device-temperature-loops, base epic-fw-fanctrl-loop-6ma-integration). Normative spec: docs/superpowers/runs/2026-09-11-per-device-temperature-loops/2026-09-11-per-device-temperature-loops-design.md, revision 4. Replace the scalar budget, CPU/GPU allocator and Mode A/B arbiter with one shared T* source and independent CPU-watt/GPU-clock DeviceLoops over their own EC sensor-group averages. Deliver bumpless entry and resume, measured-crossing one-time thermal handover, shadow caps, elapsed-time PI/slew behavior, per-device hot-guard max ratchets, paired actuator verification, qualified T* persistence, per-device calibration, additive v3 telemetry followed by deletion sweep, and the complete offline acceptance matrix. Run completion = this epic closed. The 30-minute gaming check and VR/VRAM label spike remain the user's hardware work in run.md; no physical-hardware step is automated.""",

    f"{ROOT}.1": """Spec §2.1 (rev4). Extend EcReading with cpu_group_c and gpu_group_c, each the maximum plausible reading in its device group, and with a separate reconciliation_max_c. Exact membership is the eight-label table first; carried cpu/apu and gpu_ prefix fallbacks remain device-group inputs. Only the exact ambient and charger labels are known uncontrollable. An unmatched non-device label is Unknown: keep it visible in plausible all/argmax and raw reconciliation, raise EcUnknownLabel, force Held when it is argmax, and never classify it as Uncontrollable. Report unexpected and missing expected labels at discovery and on label-set changes, logging only when the diagnostic set changes; sentinel/ENODATA on a present label is not a missing label.

The control stream accepts only finite 0 < value <= EC_PLAUSIBLE_MAX_C=110; rejected values are omitted from max_c, argmax, all, both group maxima and T* feasibility, with EcImplausible naming each label. The raw reconciliation stream accepts every finite positive value, including >110, and rounds/maximises exactly like fw-fanctrl. ec_valid still requires at least one plausible control reading. There is no jump filter. A plausible 105 C device reading remains in its group and fails safe toward the floor; a stuck known-uncontrollable reading is handled by TStarSource quarantine.

Acceptance: exact and prefix group cases; dGPU-off -> gpu_group_c None; missing/unknown-label diagnostics and change-only logging; Unknown argmax routes to Held, never Bypass; -150 and non-finite values are absent from both streams; 150 C is excluded from every control field but retained in reconciliation_max_c with EcImplausible; a raw-only sample cannot keep ec_valid true; 105 C remains the group maximum. Update synthetic hwmon fixtures so loaded dGPU data has a GPU argmax near 83 C and idle retains ambient argmax.
owns: group classification, label diagnostics, plausible control fields, ec_valid inputs, and the separate positive-only reconciliation maximum.
consumes: nothing new.""",

    f"{ROOT}.2": """Spec §2.2 (rev4). Replace the single EcAverage history with three boxcars over the same ma_interval, capped at 100 samples and retaining the carried mean-before-append behavior. The reconciliation boxcar consumes only reconciliation_max_c and is the sole stream compared with fw-fanctrl movingAverageTemperature. CPU/GPU boxcars consume only their own plausible instantaneous group maxima and expose cpu_group_ma()/gpu_group_ma().

Discard histories on Auto engagement, re-engagement from Released, calibration exit, resume, and the explicit reseed after EC MISMATCH clears. Seed reconciliation from a usable fresh view.ma_temperature; otherwise accumulate raw maxima and remain unreconciled until a full usable window and successful scored comparison. Seed each available device group independently with N copies of its own current instantaneous maximum. A missing group clears only its history and reports None; its first plausible returning sample seeds that group before ordinary averaging. view_changed, ordinary polls, strategy changes and point edits do not reseed: set_interval retains samples and trims oldest excess only when N shrinks.

Acceptance: CPU50/GPU70/socket-MA80 at every reset reports CPU50/GPU70; no usable view leaves reconciliation unseeded until a full scored window; group None and return affect only that group; mismatch-clear and resume resets work; poll/strategy/points edits preserve histories; shrink/grow retains the specified samples; a 150 C value reaches only reconciliation; the existing emulator reconciliation test stays green.
owns: all three boxcars, their seed/reseed/resize rules, per-group average API, and reconciliation state.
consumes: EcReading group fields and reconciliation_max_c (eb9.1).""",

    f"{ROOT}.3": """Spec §2.3 (rev4) DeviceLoop core, the unblocking artifact for all consumers. Add generic DeviceLoop<W|MHz>, Gains/default_gains(ma_interval), ThermalMode{Regulate,Bypass}, ActuatorState{Verified,Mismatch,Unverifiable}, Bound, Hold, Selected and DeviceDecision. tick input includes t_star, group_c, draw, floor/max, mode, actuator, dt_s, resumed and delta_tstar. Keep this module pure. Leave shadow=max as an explicit stub for eb9.4, while defining the two-candidate selector and public seed/transfer API it fills.

Thermal regulation is velocity-form PI on err=t_star-group_c at a 5 s elapsed cadence: dt_control=clamp(dt_s,0,2), with non-finite/negative as zero; accumulate, run at most once per sample with elapsed bounded to 7 s, then clear. Use u_next=u+Kc*(err-e_prev)+Kc*elapsed/Ti*err, suppress only integral motion farther into an active bound, and update e_prev even while that integral term is suppressed. On every actual T* change shift e_prev += delta_tstar before the PI update; do not call resync_error for Curve/ Held setpoint motion. This cancels only the setpoint P kick, preserving a simultaneous measured-temperature delta. resync_error is reserved for entry, resume, mismatch recovery, group recovery and explicit mode transfer. Default gains use live ma_interval: CPU τ/Ti=35 s, K=.8 C/W; GPU provisional τ/Ti=15 s, K=.02 C/MHz, θ_eff including the 40 s tail, yielding about 2.1 MHz/C at ma_interval=60.

Missing group holds last successfully applied cap (max if none); after 60 s valid control time following prior availability, request max and raise GroupLost until recovery; absent from session start has no dwell/flag and caller skips the write. Missing draw leaves thermal regulation live and reports DrawUnavailable; shadow consequences belong to eb9.4. Mismatch freezes PI/candidates until Verified, then resyncs; Unverifiable does not hold. resumed holds the pre-suspend cap, clears PI elapsed/draw/dwell baselines and performs no PI/candidate step. Wall-clock suspension never advances dwells.

Selector uses unquantised clamped candidates: requested==floor -> Selected::Floor; else requested==max -> Max; else thermal<=shadow -> Thermal; else Shadow. Thus floor/max ties retain bound identity and interior ties choose Thermal. Quantise CPU to 0.5 W; maintain a per-sample slew-limited requested state even when actuator writes are deferred. Bounds win after quantisation. Floor changes and guard downward ratchets bypass slew. A lower guard max clamps applied cap, thermal and PI state regardless of error, mode or Mismatch; recovery raises only the max ceiling.

Acceptance: first-order response within 1% and <=5% overshoot for CPU and provisional GPU defaults; clamp unwind with e_prev still updated; coincident T* and measurement changes preserve the measurement P term; default_gains reflects live interval and GPU τ/Ti=15/Kc≈2.1; dt jitter, backlog and a t=100->7300 resume produce bounded elapsed behavior with no resume step; group absent/lost/recovery and draw-unavailable thermal continuity; mismatch/Verified/Unverifiable behavior; floor/max/interior ties; per-sample slew accumulation between writes; guard clamp during Bypass and Mismatch; seed and resync have no kick.
owns: core DeviceLoop types and tick contract, thermal PI and gain defaults, elapsed/resume handling, selector/bound precedence, group/mismatch holds, quantisation/slew state, guard-max clamp semantics, and seed/resync/delta-T* APIs.
consumes: nothing new.""",

    f"{ROOT}.4": """Spec §2.3 (rev4) shadow candidate, measured-crossing handover and mode transfers; fill the DeviceLoop stub from eb9.3. shadow_target=clamp(draw+headroom,floor,max). In Regulate, both shadow directions move only while err>=0: rise at CPU headroom/s or GPU 300 MHz/s and fall at the configured rate; while err<0 both hold. In Bypass both directions run regardless of frozen T*. Missing draw freezes shadow; after 60 s valid control time request max through normal output rise slew, without a step, and on return seed shadow at current applied cap before resuming. gpu_shadow_enabled=false requests max while retaining output slew.

Before candidate motion, perform a one-time handover only for a measured hot crossing prev_group<=prev_tstar && group>prev_tstar && group>tstar, with previous decision selecting Shadow and an armed episode. Seed thermal/PI output to clamp(last_applied,floor,max), resync measured error and skip PI for that tick. Initial hot entry uses entry shadow as last_applied. Rearm only after err has remained nonnegative for 5 s of control time. Preserve latch/debounce over Held updates. A target-only sign change, repeated hot tick or draw dip cannot hand over, and draw never continuously overwrites thermal.

On Bypass entry seed shadow to last_applied, freeze thermal and suppress motion for the transition tick. On exit seed thermal to last_applied, resync error, clear PI elapsed state, suppress transition motion and initialise the hot latch from current error. Entry/exit are step-free even when thermal and shadow differed. Applied-output slew remains active with shadow disabled.

Acceptance: cool load rises and scene dips use elapsed slew/fall rates; both directions hold throughout a hot Regulate episode; Bypass shadows follow load while hot; exactly one measured crossing hands over, with 5 s rearm, and target-only changes/draw dips do not; a 60 s hot draw dip leaves thermal identical to an equal-error replay; missing draw dwell/return and gpu_shadow_enabled toggles have no output jump; Bypass entry/exit from unequal candidates are step-free; initial hot entry is bumpless; Selected::Shadow/Hold::Shadow become reachable under the core tie rules.
owns: shadow candidate and configuration use, missing-draw shadow behavior, measured-crossing episode handover, two-candidate mode transfers, and the Shadow selector/hold legs.
consumes: DeviceLoop core and transfer hooks (eb9.3); shadow Config keys (eb9.6).""",

    f"{ROOT}.5": """Spec §2.4 (rev4) TStarSource state machine and Curve path in control/tstar.rs, replacing mode.rs. Auto enters Held. Accept a persisted seed only through eb9.6's qualified TStarSeed validation; otherwise seed from the maximum available controllable-group average, or T*_ceiling when neither exists, then clamp with the shared feasibility helper. Curve entry requires fresh view, resolvable tread for snapped target duty, reconciled replica, and debounced controllable argmax for 15 s. Re-derive only when curve points or snapped duty changes. Send actual delta_tstar to both loops so DeviceLoop shifts e_prev; do not use resync_error for Curve or Held setpoint changes. An explicit upward fan-target/curve change in Curve restores thermal to max while leaving shadow and last_applied alone; output slew governs recovery.

Uncontrollable applies only to a plausible exact ambient/charger argmax: freeze T*, emit ArgmaxUncontrollable and tick both loops in Bypass. Exit to Held through step-free loop mode transfers. Unknown argmax, quarantined labels, stale/invalid views and mismatch select Held/Regulate. Released covers invalid fan/EC or watchdog release; calibration freezes the state machine externally.

Add the known-uncontrollable stuck backstop: while one label continuously holds debounced argmax, a complete fresh 300 s rolling span <=.25 C raises ArgmaxStuck(label), quarantines it from T* argmax/feasibility for the Auto session, exits Bypass to Held/Regulate and inhibits Curve and Uncontrollable while any quarantine remains. Telemetry and raw reconciliation retain it. Clear quarantine only after 30 consecutive fresh readings differ by >.5 C from the quarantine value; missing/implausible values reset recovery streak. Pending detection resets on missing, resume or loss of dominance. Independent group loops and guards remain live.

Persist current valid controlled T* with resolved strategy, sanitised requested fan target and wall timestamp on Held exit, Auto exit and at most every 60 s while dirty. Never refresh it in Released/Uncontrollable, when both groups are absent, or while quarantine, Unknown or empty-feasibility is active. Mid-session strategy/target changes rekey future writes without reloading persisted state.

Acceptance: every state/transition and 15 s gate; exact uncontrollable versus Unknown routing; unchanged view points do not rederive; Curve and Held changes pass delta_tstar rather than resync; explicit upward Curve target restores thermal=max under output slew; step-free Bypass transfers; rolling stuck detection, false-stable visibility, quarantine inhibition and 30-sample recovery; resume resets pending detection but not session quarantine; valid persistence cadence and every prohibited-save case.
owns: TStarSource states/transitions, Curve derivation and change cache, argmax debounce, known-uncontrollable quarantine, ThermalMode output, persistence meaning/cadence, and the Held slot eb9.16 drives.
consumes: DeviceLoop Hold/ThermalMode/delta-T*/transfer APIs (eb9.3/4), sensor classification and diagnostics (eb9.1), replica verdict (eb9.2), qualified state shape (eb9.6), existing view/curve/table.""",

    f"{ROOT}.6": """Spec §2.8 and Config portions of §2.3 (rev4). Add positive-sanitised shadow_headroom_cpu_w/gpu_mhz (>=1 W/>=30 MHz), shadow_fall_rate_cpu/gpu (>=.05 W/s/>=1 MHz/s), gpu_shadow_enabled=true, per-device optional Gains overrides, and gpu_max_mhz default 3090 bounded by gpu_floor_mhz. Add cpu_hot_c default 90 sanitised independently to [82, CPU_TRIP_C-1]=[82,94]; do not derive its lower bound from GPU settings. CPU exit/recovery remain cpu_hot_c-3/-5. Rehome CPU_MAX_W and CPU_MAX_W_FLOOR in config.rs before allocator deletion. Remove gpu_max_w with one migration warning; no shadow_band key.

PersistedState removes lut/loop_gains and adds keyed cpu_gains/gpu_gains maps, paired WarmStartEntry{cpu_cap_w,gpu_lock_mhz}, and t_star_last_good: Option<TStarSeed{strategy,fan_target_rpm,value_c,saved_at_unix_s}>. Validate finite positive in-range temperature, nonempty strategy and sanitised target. At use, reject wrong strategy/target, expired >6 h, future timestamps and legacy scalar; clamp accepted value to current bounds. Preserve the original timestamp when unrelated state is saved. Ignore old lut/loop_gains and bare warm-start values with one migration log each; absent fields stay valid.

Acceptance: every numeric floor/bound, cpu_hot_c=82 -> CPU ceiling80 while default GPU ceiling86, independent CPU/GPU sanitisation, old-config warning, and CPU max bound after rehome; keyed maps and paired warm start round-trip; qualified T* seed fresh/matching accepted, wrong key/expired/future/legacy rejected, and unrelated saves never renew age; old persisted fields are ignored with exactly one log line each.
owns: Config declarations/defaults/sanitisers and CPU-bound rehome; PersistedState maps, WarmStartEntry, qualified TStarSeed and migrations.
consumes: Gains (eb9.3).""",

    f"{ROOT}.7": """Spec §2.5 (rev4) controller wiring. AutoState owns TStarSource, two DeviceLoops, three-boxcar replica, steady/warm-start state, CPU five-valid-sample draw mean, per-device write cadence and GPU command-pair history. on_auto_sample orders replica -> tstar (with previous-tick Hold) -> guards -> device ticks -> writes -> telemetry. Pass dt_s/resumed/delta_tstar, per-group averages, CPU mean/GPU reported SM clock and prior paired actuator verdicts. Resume holds pre-suspend caps, clears PI/draw/boxcar/dwell/pairing elapsed state, reseeds, resyncs errors and immediately reasserts; its first GPU sample is Unverifiable.

Apply MaxRatchet to each loop's max. GPU guard uses die; CPU Tctl uses a 3-consecutive-sample debounce. Every downward guard move clamps thermal/PI and applied cap independent of group error, Bypass or Mismatch, and writes immediately; recovery raises only the ceiling. Preserve emergency release, watchdog, fences, stock restore/read-back and mismatch rewrite behavior per device.

Write only a changed quantised cap after that device's 2 s minimum interval, except floor, downward guard, Released and resume reasserts. DeviceLoop continues accumulating per-sample slew/request state between writes; send the latest pending request when due. last_applied means latest successful command. Pair GPU verification to sample acquisition: record every completed command's monotonic time/generation, score against the latest command completed before the sample, and return Unverifiable without a match. For the first 1 s interval after a downward change allow max(paired command, immediate predecessor)+30 MHz; afterward paired command+30 only. Lock changes do not reset strikes. One-command-lag compliant hardware must not strike during 105 MHz/s descent; ignoring hardware still trips. CPU synchronous read-back remains paired.

Auto entry uses qualified T* seed rules, thermal=max, and shadow=draw+headroom after the five-sample mean is available; until then thermal-only. Initial hot entry uses the one-time handover. Warm start is advisory: thermal=max(recorded,draw+headroom) when usable, never below entry headroom. Record the warm pair only after both groups and fan meet the steady window; keep duty/RPM refinement. Resolve gains Config override > fitted current strategy/interval > live defaults and report source. Remove the NOT CALIBRATED entry gate. Freeze loops/TStar/caps throughout calibration and reseed at caps in force on exit. An absent-from-start GPU receives no lock until its group appears; GroupLost max release is written. Manual/Monitor/release/quit/emergency behavior remains, wired to both devices.

Acceptance: bumpless cold/warm/hot entry and delayed draw; dt/resume no-step/reassert; live CPU mean; gains precedence/source and strategy/interval change; persistence load/write cadence; calibration freeze/unfreeze; 2 s writes with per-sample pending slew; floor/guard/release exceptions; guard clamping when group cool and under Mismatch; paired compliant-lag/no-match/ignoring-card verifier cases; mismatch, watchdog, fences, restore and emergency paths; warm record excludes fan-out-of-tolerance; Uncontrollable ticks both loops in Bypass; exhaustive wiring sweep includes resume/emergency.
owns: AutoState composition and ordering, tick inputs/dt/resume plumbing, draw windows, guard/debounce wiring, paired verification, write cadence/pending dispatch, entry/warm/gains/persistence/calibration wiring, decision emission call site, and all two-device lifecycle paths.
consumes: eb9.1-6, MaxRatchet (eb9.15), Held driver/flags (eb9.16), additive v3 schema (eb9.9).""",

    f"{ROOT}.8": """Spec §2.6 (rev4). Replace calibration with a per-device step test run twice; lut_sweep is deleted later by eb9.12. The controller owns the external freeze; this bead owns settle/step/restore writes through the normal actuator paths. Settle with both caps held, controllable argmax, both groups flat <=.5 C over 60 s and fans flat <=150 RPM over 20 s, capped at 600 s with a skip reason naming the failed condition and duration. At every calibration sample, EC max>=95, GPU die>=gpu_hot_c or 3-sample CPU Tctl>=cpu_hot_c aborts; guards win, restore both pre-step caps and finish Noted.

CPU: +15 W from settled cap, clamped to cpu_max_w, GPU held. GPU: +500 MHz, clamped to gpu_max_mhz, CPU held. Use the common 360 s fit window, including the slower CPU path; fit_fopdt/derive_gains in native units and use fitted theta as-is. Reject if the other group moves by more than max(1 C,.2*primary delta), or the primary response fails the existing magnitude rule. Save accepted gains in per-device map at <strategy>:<ma_interval>. A rejected device keeps its currently resolved config/fitted/default gains and reports why. calibrated_at stamps the run; NOT CALIBRATED is informational.

Acceptance: settle success/each timeout reason; every overtemperature abort restores caps; each cross-term/magnitude rejection and a coupled response below threshold; CPU and GPU fit/write/restore paths; keyed saves and strategy fallback; calibration freeze preserves loop/TStar state and exit reseed has no cap step.
owns: per-device step runner, settle/abort/rejection rules, calibration writes/restores and keyed fitted-gain saves.
consumes: Gains/bounds (eb9.3), state maps (eb9.6), group boxcars (eb9.2), controller freeze and actuator paths (eb9.7).""",

    f"{ROOT}.9": """Spec §2.9 (rev4), additive-before-sweep telemetry. Introduce decision schema v3 fields t_star, tstar_state, per-device {group_c,err_c,thermal,shadow,cap,selected,hold,gains_source}, cpu_limit_w, gpu_max_mhz and flags including ArgmaxUncontrollable, ArgmaxStuck(label), EcUnknownLabel, EcImplausible, EcUncontrollableUnavailable, GroupLost, DeviceUnreachable, TargetUnreachable and SteepCurve. Add cpu_group_c/gpu_group_c to sample lines. Keep legacy Budget*/pi_target_w/alloc_*/demand_*/freeze fields deprecated and populated until controller rewiring lands; eb9.12 removes them afterward so every intermediate leaf compiles.

Acceptance: additive schema compiles against the old emission site; every_telemetry_field_is_populated passes before and after controller rewiring; round-trip real DeviceDecision values for every Selected/Hold and gains_source; all new sensor/TStar flags serialise with label/polarity details; sample group fields populate independently, including None.
owns: additive v3 decision/sample schema and field-by-field README/docs telemetry documentation.
consumes: DeviceDecision types (eb9.3/4), TStar state/flags (eb9.5/16), EcReading groups/flags (eb9.1).""",

    f"{ROOT}.10": """Spec §4 (rev4) plant and fault API. ChainedPlant gains CPU and GPU group nodes with 0.1 C/C cross-coupling. CPU nominal τ=35 s, K=.8 C/W and scripted single-sample package-power bursts over sustained cap. GPU nominal raw-group τ=15 s (provisional from 11.26/13.89 s replay fits; individual member poles span about 35-43 s), K=.02 C/MHz via the .4 C/W heat path, and parameters supporting τ={8,15,25,50}, K={.01,.02,.03}, θ_eff={45,90,135}. The GPU heat equation must use its .4 C/W path.

Model draw_w=load_level*P_full(clock) using the September piecewise points (1197,49.3), (1402,53.5), (1612,64.2), (1807,75.9), (1995,90.8), (2143,99.4), flat to (3090,100) and linear to (1000,45). reported SM clock=min(lock,clock_at_power_limit(load)); full-load clock plateaus above the knee. Script independent GPU die and CPU Tctl.

Fault/load API covers dGPU absent, either group lost mid-run with whole EC valid, 105 C stuck device labels, 105 C ambient/charger labels, 150 C implausible labels retained in raw reconciliation only, unknown labels, ambient/charger -150 sentinel pair, fan outage, EC invalid, stale/changed view, draw unavailable windows, resume/backlog dt, square-wave loads, target/curve switches, guard group-hot/group-cool branches, and GPU verifier behavior (timestamped compliant one-command lag or ignoring card). Emulator exposes both plausible control fields and positive-only raw reconciliation exactly like eb9.1.

Acceptance: open-loop plateau/knee and reported clock behavior; nominal GPU node reaches about 82 C from 42 C at 100 W; nominal τ=15 and each robustness parameter selectable; each fault observable at sample boundary in both control/raw streams; CPU burst and square waves reproduce scripted amplitudes; paired verifier fake produces compliant lag and persistent-ignore histories.
owns: two-node parametric plant, clock/draw/heat model, raw/control EC emulation, verifier fake and all fault/load scripts used by acceptance beads.
consumes: EcReading output shape (eb9.1).""",

    f"{ROOT}.11": """Spec §4 (rev4) sims 1-4 and 10 plus shared helpers. Sims 1-3 CPU-heavy/GPU-heavy/both-heavy require stressed groups at T*, unstressed devices at shadow above draw, and fan within ±150 RPM for >=90% of a 30-min converged window. CPU robustness remains K/τ/θ ±50%. GPU robustness is the full cross product τ={8,15,25,50}, K={.01,.02,.03}, θ_eff={45,90,135}, with controller defaults fixed at provisional τ/Ti=15 and Kc≈2.1; each combination runs the relevant CPU/GPU/both load cases.

Sim 4 runs GPU light->heavy from cold and warm. Cold: 1000 MHz upward slew <=5 s, fan crest <=target+250, group overshoot <=4 C, no GPU HOT, CPU cap unchanged. Warm overshoot <=2 C. Define settling as ±1 C within 3λ after the first downward knee crossing plus θ_eff. Record D from handover and the minimum sustained hot error over the plateau, and separately assert crossing <=D/(Kc/Ti*e_min)+one PI period. If the response starts below the knee D=0 and start is the crossing time; never use the earlier upward onset crossing or divide by zero when no hot response. Repeat cold step with gpu_shadow_enabled=false and with draw absent >60 s: record full ~947 MHz travel bound, require the same thermal/guard/post-crossing bars and no jump on disable/dwell/return.

Sim 10 adds square-wave 60/300 s ±4 C loads, Curve T* down-and-back, corresponding Held excursion, and a 60 s draw dip beginning while each group is hot. Curve target return reaches prior cap within 10% in 3λ; Held recovery uses 3λ_inner after T* itself returns. Hot-dip PI state must match an identical thermal-error replay without the draw dip; any purely shadow-limited 1000 MHz recovery is <=5 s after err>=0. Apply a shared period-agnostic no-relay check to every 30-min window: no unexplained sustained cap/fan oscillation above 4 W/150 MHz/100 RPM at periods 30 s-20 min.

Acceptance: every bar is named and reports timing origin; traces prove one measured handover, e_prev delta shift without loss of measurement P response, per-sample slew between writes, normal/disabled/missing-draw knee legs, nominal and full GPU matrix, CPU ±50% matrix, Curve/Held/draw-dip recovery and reused no-relay helper.
owns: sims 1-4/10, robustness matrices, downward-knee/travel/recovery helpers, converged-window and no-relay helpers.
consumes: plant/fault API (eb9.10), loops (eb9.3/4), TStarSource (eb9.5/16), controller wiring (eb9.7).""",

    f"{ROOT}.12": """Spec §3 (rev4) terminal deletion sweep across path and basename in source, tests, manifests, fixtures, README and docs. Delete budget.rs/Freeze, allocator.rs and pinned margins/tests, spike_antiwindup.rs, mode.rs after its surviving logic moves, lut.rs, lut_sweep.rs, GPU watts inner loop, DEMAND_MARGIN_W_*, gpu_max_w, gpu_share_override, PersistedState lut/loop_gains, old LUT fixtures/sims, and all obsolete budget/split/LUT/Mode A-B/shadow_band/pinned documentation. Rehome CPU bounds and all moved semantics before deletion. Remove the deprecated Budget*/pi_target_w/alloc_*/demand_*/freeze telemetry fields only after eb9.9's additive v3 schema and eb9.7 emission site have landed.

Acceptance: path-and-basename searches for every deleted item have zero non-historical hits; no stale superseded behavioral claim survives current docs; every v3 field remains populated; cargo test and cargo clippy --all-targets -- -D warnings pass after the sweep.
owns: terminal deletions and stale-reference cleanup.
consumes: all preceding implementation leaves; this remains the terminal sweep.""",

    f"{ROOT}.13": """Spec §2.9 (rev4) TUI. Render T*, Held/Curve/Uncontrollable/Released, both group temperatures/errors, both candidate and applied caps with T/S/F/M binding identity, per-device Hold and gains source, and flags including device/target unreachable, ArgmaxUncontrollable, ArgmaxStuck(label), EcUnknownLabel, EcImplausible and EcUncontrollableUnavailable. Distinguish GroupUnavailable, GroupLost, DrawUnavailable, Bypass and Mismatch. Keep c/g/a/p/k/q keys and remove budget/split/LUT panels.

Acceptance: fixtures render every TStarState, Hold, Selected, gains source and new flag/label; exhaustive enum matches make a new variant a compile failure; missing values remain legible.
owns: TUI presentation of v3 control/sample state.
consumes: v3 schema (eb9.9), DeviceDecision (eb9.3/4), TStar states/flags (eb9.5/16).""",

    f"{ROOT}.14": """Spec §4 (rev4) sims 5-9 and 11 plus enum-derived configuration smoke. Sim 5: heavy-load Auto entry changes draw by <=1.5 W/30 MHz in 10 s, starts each cap at or above draw+headroom and does not drop fans, including qualified-seed and no-draw entry variants. Sim 6: converged curve loss on EC-autofan (0.25 schedule, λ_eff=96 min) and quiet16 (1x, 72 min) returns fan within ±150 RPM by 3λ_eff with no relay; Curve return has no cap step. Restart variants cover matching fresh qualified seed and wrong strategy/target, expired, future and legacy records: only matching fresh is accepted, and each group seeds from its own instantaneous maximum rather than socket MA.

Sim 7: unreachable GPU never alters CPU and reports DeviceUnreachable. Sim 8 behaviorally reaches every TStarState/Hold/Selected/flag from enum-derived checklists, including unknown argmax, known-uncontrollable Bypass, absent dGPU without GroupLost, mid-run loss/release/recovery, DrawUnavailable before/after dwell, clamps/ties, mismatch/unverifiable, quarantine, empty-feasibility and Released.

Sim 9 drives five-minute GPU and CPU guard episodes. Ratchets reach floor; GPU max recovers only with die<=hot-4, CPU with Tctl<=hot-5; one CPU spike does not trip but a 3-sample streak does. For both, start the 3λ recovery bar only when temperature remains below recovery threshold, load/T* again match the prior feasible condition, and max has reopened to the pre-episode cap. Report guard-clear, recovery-gate and ceiling-reopened times separately. Cover group-already-hot and group-still-cool branches, proving guard clamps thermal/PI independent of group error. A compliant one-command-lag verifier has zero strikes/releases during descent; a separate ignoring-card leg trips. Bound the first full CPU .5 W rise under sustained positive e_min by .5/(Kc/Ti*e_min)+one PI period+write interval; make no unconditional 60 s floor-exit promise.

Sim 11: 105 C GPU label from start and t=5 min reaches floor; separately bound floor travel from actual seeded output by D/(Kc/Ti*e_min)+PI/write latency while err<=-e_min, then require DeviceUnreachable within BOUND_HOLD_S after floor. After unstick, feasible load/target and boxcar flush, recover cap/temperature within 3λ. A 150 C label is excluded from control but retained in emulator/replica raw reconciliation for >=4 scored views with no gate-caused EC MISMATCH or direct error/cap change. Ambient/charger 105 C start/mid-run variants raise ArgmaxStuck within debounce+300 s, quarantine into Held/Regulate, cannot re-enter Bypass, then clear after >.5 C for 30 samples with guards still live. A -150 ambient/charger pair with valid groups keeps EC valid, raises EcUncontrollableUnavailable and produces a finite collapsed Held interval until plausible return.

Acceptance: every clause is a named assertion; sim6 legs have distinct λ_eff clocks; sim9 uses recovery eligibility rather than guard-clear deadline; sim11 separates floor travel, post-floor flag dwell, boxcar flush and post-flush recovery; smoke is derived from enums/flag registry and fails on an unreached addition.
owns: sims 5-9/11 and behavioral smoke checklist.
consumes: plant/fault/verifier API (eb9.10), helpers (eb9.11), loops/TStar/controller (eb9.3-7/16), ratchets (eb9.15).""",

    f"{ROOT}.15": """Spec §2.5 step 3 (rev4). Implement pure generic MaxRatchet for both hot guards. While active, reduce max by GPU 105 MHz or CPU 2 W per valid-control sample to floor. Recover at half rate only while GPU die<=gpu_hot_c-4 or CPU Tctl<=cpu_hot_c-5, up to configured max. Controller owns CPU's 3-sample entry debounce. On every downward ratchet, DeviceLoop clamps thermal output, PI state and applied cap to the new max independent of device-group error, candidate selection, Bypass or ActuatorMismatch. This is thermal guard evidence, never draw/shadow tracking. On recovery raise only the ceiling.

Acceptance: down rate/floor/idempotence, half-rate recovery and temperature gate, flapping around exit cannot create a max limit cycle, CPU/GPU units, guard while averaged group remains cool, and integration hook proving thermal/PI clamp during Bypass/Mismatch while recovery leaves PI unchanged.
owns: MaxRatchet and device-specific rates/gates; guard-max clamp contract consumed by DeviceLoop/controller.
consumes: device bounds/PI clamp hook (eb9.3), Config bounds/cpu_hot_c (eb9.6), existing GPU/CPU guard states.""",

    f"{ROOT}.16": """Spec §2.4 Held driver and feasibility (rev4). Drive Held T* with velocity-form RPM PI: err_rpm=fan_target-FAN_SMOOTH_N=5 tail mean of max fans, raw fallback on outage; 5 s elapsed cadence, λ_held=1440 s, Kc≈2.9e-4 C/RPM at reference slope, Ti=35 s, <=.5 C output step. Schedule Kc by slope_ref/max(slope_at(T*),slope_ref), clamped [.25,1], using .25 with no resolvable curve; λ_eff=λ_held/schedule. Pass every actual Held delta_tstar to DeviceLoop so it shifts e_prev; do not resync on incremental Held motion.

Use T*_ceiling=min(cpu_hot_c-2,gpu_hot_c-2). For nonempty plausible known non-quarantined uncontrollables, raw_floor=max+5 and floor=min(raw_floor,ceiling), raising TargetUnreachable(high) when raw exceeds ceiling. If empty, floor=ceiling and raise EcUncontrollableUnavailable; restore immediately on valid return. Clamp/conditionally integrate directionally. Previous-tick Hold blocks upward only when every device is in {Clamp(Max),Shadow,Bypass,ActuatorMismatch,GroupUnavailable}; DrawUnavailable is in neither set because thermal regulation stays live. Block downward only when every device is in {Clamp(Floor),Bypass,ActuatorMismatch,GroupUnavailable}. Match Hold exhaustively.

Raise TargetUnreachable(low/high), SteepCurve, and per-device DeviceUnreachable after 60 s valid control time at Clamp(Max) with group below T* (informational) or Clamp(Floor) with group above T* (real). Resume/wall gaps restart bound dwell. Quarantined/Unknown readings are excluded from feasibility.

Acceptance: elapsed RPM PI cadence/step/output clamps and no windup; .25/1 slope schedule; empty and raw>ceiling intervals are finite and flagged; cpu_hot_c=82 and default GPU bounds; DrawUnavailable+Shadow does not block upward, all other directional combinations and one-sample Hold lag; Held delta shift preserves measured-temperature proportional response; each target/device flag and post-floor 60 s dwell; resume resets dwell.
owns: Held RPM PI/schedule, feasibility helper and flags, directional outer anti-windup, DeviceUnreachable dwell, and exported TStar decision flags.
consumes: TStarSource Held slot/curve slope (eb9.5), Hold and delta-T* APIs (eb9.3/4), plausible known non-quarantined readings (eb9.1/5), fan inputs.""",

    f"{ROOT}.17": """Revision-4 root integration sweep after every leaf and deletion sweep land. Exercise end-to-end on fakes/plant: fresh and qualified-seed Auto entry, Curve regulation, Unknown/quarantined/empty-feasibility Held paths, known-uncontrollable Bypass, curve loss/return, draw/group loss/recovery, resume with long wall gap, both guards including group-cool clamp, paired compliant/ignoring GPU verification, calibration freeze/two steps/gain save, and daemon restart restoring keyed gains, paired warm start and qualified TStarSeed.

Sweep all wiring: every Config key consumed; every StatusFlag raised, serialised and rendered; every v3 sample/decision field populated from live values; every Hold/Selected/TStarState reaches telemetry/TUI; every elapsed-time/delta-T*/resync call uses the specified path; per-sample requested slew and 2 s writes remain distinct; no deleted symbol/path survives outside explicitly historical material. Reuse the full acceptance suite, including GPU τ/K/θ matrix, downward-knee timing, recovery-eligibility guard bars and raw-reconciliation/quarantine legs. Add only small missing integration coverage inline and file a blocker for any large discovered gap.

Acceptance: complete flows pass; enum/flag/config/telemetry/TUI/deletion sweeps have zero open items or a filed blocker; cargo test and cargo clippy --all-targets -- -D warnings pass; append the final checklist evidence to the spec's Post-Implementation Notes.
owns: terminal end-to-end validation and small integration-only fixes.
consumes: every leaf.""",
}


def preserved_lines(description: str) -> list[str]:
    return [
        line
        for line in description.splitlines()
        if line.startswith("blocked-by ") or line.startswith("Files: ")
    ]


def compose(body: str, original_description: str) -> str:
    suffix = preserved_lines(original_description)
    return body.strip() + (("\n" + "\n".join(suffix)) if suffix else "")


def render_tree(issues: list[dict]) -> str:
    lines = [
        "# Settled task tree — epic fw-fanctrl-loop-eb9 (descriptions rewritten wholesale against spec revision 4)",
        "",
    ]
    for issue in issues:
        blockers = sorted(
            dep["id"]
            for dep in issue.get("dependencies", [])
            if dep.get("dependency_type") == "blocks"
        )
        deps = sorted(
            f'{dep["id"]} ({dep.get("dependency_type", "blocks")})'
            for dep in issue.get("dependencies", [])
        )
        lines.extend(
            [
                f'## {issue["id"]} [{issue["issue_type"]}] {issue["title"]}',
                f'status: {issue["status"]}',
                f"blocking deps: {blockers}",
                f"all dependencies: {deps}",
                issue["description"],
                "",
            ]
        )
    return "\n".join(lines)


def main() -> None:
    before_live = [show(issue_id) for issue_id in IDS]
    if SNAPSHOT.exists():
        before = json.loads(SNAPSHOT.read_text())
    else:
        before = [issue_snapshot(issue) for issue in before_live]
        SNAPSHOT.write_text(json.dumps(before, indent=2, sort_keys=True) + "\n")

    before_by_id = {issue["id"]: issue for issue in before}
    if set(before_by_id) != set(IDS) or set(BODIES) != set(IDS):
        raise SystemExit("revision-4 description or snapshot ID set is incomplete")

    expected_graph = {issue["id"]: graph_signature(issue) for issue in before}
    live_graph = {issue["id"]: graph_signature(issue) for issue in before_live}
    if live_graph != expected_graph:
        raise SystemExit("live title/status/priority/type/parent/dependency graph differs from snapshot")

    expected_descriptions = {
        issue_id: compose(BODIES[issue_id], before_by_id[issue_id]["description"])
        for issue_id in IDS
    }
    for issue_id in IDS:
        bd("update", issue_id, "--description", expected_descriptions[issue_id])
        print("rewrote", issue_id, len(expected_descriptions[issue_id]))

    after = [show(issue_id) for issue_id in IDS]
    after_graph = {issue["id"]: graph_signature(issue) for issue in after}
    if after_graph != expected_graph:
        raise SystemExit("description rewrite changed title/status/priority/type/parent/dependency graph")
    actual_descriptions = {issue["id"]: issue["description"] for issue in after}
    if actual_descriptions != expected_descriptions:
        raise SystemExit("one or more live descriptions do not match revision-4 source")
    for issue in after:
        original_suffix = preserved_lines(before_by_id[issue["id"]]["description"])
        current_suffix = preserved_lines(issue["description"])
        if current_suffix != original_suffix:
            raise SystemExit(f'preserved blocked-by/Files lines changed for {issue["id"]}')

    TREE.write_text(render_tree(after))
    if TREE.read_text() != render_tree([show(issue_id) for issue_id in IDS]):
        raise SystemExit("rendered task tree is not an exact view of live bd data")
    print(f"validated {len(after)} descriptions; graph/status and preserved lines unchanged")
    print("wrote", SNAPSHOT)
    print("wrote", TREE)


if __name__ == "__main__":
    main()

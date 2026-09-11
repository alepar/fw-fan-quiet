# Per-device temperature loops

Date: 2026-09-11 · Status: draft (revision 2, after design roast iteration 1) · Supersedes §2.4 and §2.5 of
[2026-09-07-fw-fanctrl-loop-design.md](../../specs/2026-09-07-fw-fanctrl-loop-design.md) (its
§2.2, §2.6, §2.8, §2.9 and §3.4 carry over) · Seed:
[2026-09-11-per-device-temperature-loops-seed.md](../../specs/2026-09-11-per-device-temperature-loops-seed.md)
· Run: `docs/superpowers/runs/2026-09-11-per-device-temperature-loops/`

## Goal

The fan target is met by regulating each device's own EC temperature group toward one shared
setpoint T\* — no scalar power budget, no CPU/GPU split. Engaging Auto under a running game is
bumpless; a device the load does not stress sits at a shadow cap just above its draw, never
uncapped; a device pinned at its cap is never held back by the other device's unused allocation;
a load jump becomes a ramp of seconds rather than a fan overshoot.

## Decisions taken with the user (2026-09-11)

1. The scalar budget, the demand split and the any-axis demand-limited halt are deleted, not
   repaired. Field runs `run-1789067819` and `run-1789139478` showed the split starving a
   pinned CPU while an overfed GPU under-drew and the halt vetoed every upward PI step.
2. Each device is regulated on the **max over its own EC sensor group** — the same readings
   fw-fanctrl's argmax is taken over — not on the die temperature.
3. The GPU loop drives the **max-clock lock directly**. The clock→watts LUT, its sweep and the
   watts inner loop go.
4. **Shadow cap + override control:** every device carries `draw + headroom` as a second cap
   candidate; the applied cap is the min of the thermal and shadow candidates, with tracking so
   neither integrator winds while the other is selected. A GPU clock lock is a frequency
   ceiling, so a card locked at 2 GHz running 80 % duty does the same work at a lower V/F point
   than one boosting to 3 GHz at 56 % — the GPU shadow cap binds continuously and is expected to pay for
   itself in watt-hours — **an unmeasured claim** (roast d1): it is inert while the card is
   power-limited, and the field replays show the card mostly below the knee; `gpu_shadow_enabled`
   exists so the user can A/B it during the hardware check. On the CPU `ryzenadj` caps average
   power, so the win is boost-peak clipping only.
5. **Halt = hold.** Conditional integration freezes the integrator at its last value; it is
   never zeroed.
6. **No trim with a live curve.** T\* is exactly the curve-derived value. Without a curve, a
   third PI on the fan-RPM error drives T\* from the last good value. The Mode A / Mode B
   arbiter collapses into this "T\* source".
7. **Per-device step test** for gains, replacing the single step test and the LUT sweep.
8. The 30-minute gaming check on the user's machine is the final acceptance and is the
   user's; no real-hardware step is automated.

## Facts this design rests on

- The EC fan curve and fw-fanctrl both act on the **max** over the readings
  `framework_tool --thermal` prints; those are the eight `cros_ec` hwmon sensors the daemon
  reads (`docs/research/05-fw-fanctrl-loop.md` §1, measured 2026-09-09/11):

  | framework_tool | hwmon label | group |
  |---|---|---|
  | F75303_Local | `ambient_f75303@4d` | uncontrollable |
  | F75303_DDR | `charger_f75303@4d` | uncontrollable |
  | F75303_CPU | `cpu@4c` | CPU |
  | APU | `apu_f75303@4d` | CPU |
  | dGPU VR | `gpu_vr_f75303@4d` | GPU |
  | dGPU VRAM | `gpu_vram_f75303@4d` | GPU |
  | dGPU AMB | `gpu_amb_f75303@4d` | GPU |
  | dGPU temp | `gpu_temp@40` | GPU |

  **Spike (one load test, before the group sets are frozen):** at idle the VR and VRAM values
  looked swapped between the two tools (42/46 vs 44.85/40.85); pin the label-to-label mapping
  under GPU load. The group is a max, so a swap inside the GPU group changes nothing; a swap
  across groups would.
- `gpu_vr` trails the die by 40 s+ and takes the argmax ~13 s into GPU-only load; fw-fanctrl
  averages the max over 60 s (`quiet16`). The GPU group's effective dead time is therefore
  longer than the CPU group's.
- Under a 100 W gpu-burn the die settles at 82–83 °C on `quiet16`; NVML slowdown 89, shutdown
  92; hence the GPU hot guard at 88 / exit 86 and `GPU_TRIP_C` 91 (unchanged).
- Stock CPU limits on the balanced profile read 45 W slow / 65–71 W fast / 45.3 W STAPM; the
  daemon's write is slow+STAPM at the cap with fast at `fast_limit_mw` (53 W) — unchanged here.
- Field 2026-09-11 (`run-1789139478` t≈1428): a few-thread game drew 34 W at 7 % CPU
  utilisation; the split's utilisation fallback read that as "no demand". No utilisation-based
  demand score survives this design.

## 1. Architecture

```
Sample (1 Hz) ──▶ EcReplica (§2.2 boxcar, now per GROUP)
                   ├─ cpu_group_c = boxcar(max(cpu@4c, apu))                 ──▶ DeviceLoop<Cpu> ──▶ ryzenadj slow/stapm cap (W)
                   └─ gpu_group_c = boxcar(max(gpu_vr, gpu_vram, gpu_amb, gpu_temp)) ──▶ DeviceLoop<Gpu> ──▶ NVML max-clock lock (MHz)
TStarSource ──▶ T* (one value, both loops)
   ├─ Curve:    view fresh + curve valid + replica reconciled → T* = tread temperature for duty(fan_target)   (§2.6 unchanged)
   ├─ Held:     no curve → T* = last good T* moved by the RPM PI (fan error → T*)
   └─ Released: fan/EC invalid, watchdog trip → caps released to stock (as today)
```

Two instances of one `DeviceLoop` type, generic over the device's unit (watts or MHz), each
owning its thermal PI, shadow cap and hold state. The controller becomes wiring: samples in,
T\* in, two caps out, plus the existing guards, watchdog, verification and restore paths.
`controller.rs` (7100 lines today) loses the budget/allocator/arbiter plumbing; the loop math
lives in `control/device_loop.rs` and the T\* logic in `control/tstar.rs`.

## 2. Components

### 2.1 `sensors/ec.rs` — sensor groups (change)

`EcReading` gains `cpu_group_c: Option<f64>` and `gpu_group_c: Option<f64>`: the max over the
**plausible** readings of each group's labels, `None` when the group has no plausible reading
(dGPU unpowered → GPU group `None`). Plausibility (roast d1): a reading is dropped from the max
when it is ≤ 0, > `EC_PLAUSIBLE_MAX_C = 110`, or jumps more than `EC_MAX_JUMP_C = 15` °C from
that label's previous sample; a dropped reading raises the informational `EcImplausible` flag
naming the label. A single stuck-high sensor therefore cannot drive a device to its floor for
a session. Group membership is a fixed label list in `ec.rs` (the table
above); `is_controllable()` becomes `group().is_some()`. `argmax` and `all` stay for the T\*
source's feasibility check (§2.4) and telemetry.

**owns:** the group label sets and `cpu_group_c`/`gpu_group_c`. **consumes:** nothing new.

### 2.2 `EcReplica` — per-group boxcar (change to §2.2)

The §2.2 replica becomes three boxcars over the same `ma_interval`: the argmax replica it is
today (reconciliation still compares it to fw-fanctrl's `movingAverageTemperature`), plus one
per group. All three are seeded, reseeded and invalidated together exactly as §2.2 states
(auto entry, `view_changed`, resume, `EC MISMATCH`). A group that reads `None` clears its own
boxcar and reports `None`; the loop for that device then holds its cap (§2.3 step 6).

### 2.3 `control/device_loop.rs` — `DeviceLoop` (new)

One struct, instantiated for CPU (units W) and GPU (units MHz). Inputs per 1 Hz tick:
`t_star`, `group_c: Option<f64>`, `draw: Option<unit>` (CPU: `cpu_pkg_w` when RAPL has a valid
delta, `None` on the first sample and after a counter reset; GPU: `gpu_sm_mhz` when
`gpu_mhz_valid`, with `gpu_util_pct` and `gpu_w`), `floor`, `max`, and
`actuator: ActuatorState { Verified | Mismatch | Unverifiable }` from the previous write's
read-back. Output: the cap to apply and a `DeviceDecision` record for telemetry.

**Two candidates, one selector, and a parking rule (roast d1 Blocking 1).** The thermal PI is
slow by design (a 200–360 s closed-loop constant); the shadow ramps in seconds. A min-selector
with the thermal candidate always live would pin the cap at the thermal PI's rate whenever the
device is cool — exactly the case the shadow exists for. So the thermal candidate is **parked**
while the device is cool and **active** only inside the band:

1. `err = t_star − group_c`. If `group_c` is `None` → hold the last applied cap (or `max` when
   none was ever applied), `hold: GroupUnavailable`; see the dwell rule below. If `draw` is
   `None` → hold the last applied cap, `hold: DrawUnavailable`, no seeding, no shadow update.
2. **Thermal candidate.** Active iff `err ≤ shadow_band_c` (the device is within the band below
   T\*, at T\*, or above it). When active: velocity-form PI on `err` with `Gains { kc, ti_s }`,
   `PI_PERIOD_S = 5` (the integrator runs every 5th sample; the selector runs every sample),
   output clamped to `[floor, max]` with **directional conditional integration** at the clamps
   (the `sat_dir` rule from `gpu_pid.rs`: an error pushing further into an active clamp does not
   integrate; one pulling out of it does). There is **no** "integrate only when selected" gate.
   When parked (`err > shadow_band_c`): the thermal candidate reports `max` and its integrator
   state is **tracked to the applied cap** every tick, so the hand-over into the band starts
   from the applied cap (bumpless, no windup).
3. **Shadow candidate.** `shadow_target`:
   - device **pinned** (draw within `pin_margin` of the applied cap — CPU 1.5 W, the existing
     `CPU_PINNED_MARGIN_W`; GPU: reported clock within 30 MHz of the lock, the existing
     `GPU_PINNED_MARGIN_MHZ`, with utilisation above `GPU_PINNED_UTIL_PCT`, **or** the card at
     its power limit — `gpu_w ≥ 0.95 × P_limit` with utilisation above the floor — since a
     power-limited card runs below any lock above the knee) **and** `err > shadow_band_c`
     (cool) → `applied + headroom` — one headroom step up;
   - pinned but **not** cool → hold at `applied` (no rise: the thermal candidate is active and
     owns the cap inside the band);
   - not pinned → `draw + headroom`.
   `shadow` moves toward `shadow_target` at once when rising (the GPU rise is slew-limited to
   `headroom` per second — `GPU_RISE_SLEW = 300 MHz/s` — a deliberate widening of the 105 MHz/s
   fall/steady slew so the ramp knob is real), and when falling at `shadow_fall_rate`, **except**
   that when `err < 0` (the group is above T\*) the shadow falls to `draw + headroom` at once —
   the slow fall exists to ride out scene dips, not to prolong an overshoot. Clamped
   `[floor, max]`.
4. **Selector.** `cap = min(thermal, shadow)` with the parked thermal contributing `max`;
   `selected ∈ {Thermal, Shadow, Floor, Max}` names which bound produced it. **Tracking
   (external reset):** whichever candidate is not selected has its state set to `cap` each tick
   — the shadow's current value, or the thermal integrator's `u`/`v` — so neither winds while the
   other binds. Inside the band both candidates are live and the thermal PI's own increment is
   what limits any further rise, which is the intended behaviour near T\*.
5. **Quantise and slew.** 0.5 W grid on the CPU; on the GPU `GPU_RISE_SLEW = 300 MHz/s` when the
   shadow is selected and rising, `105 MHz/s` otherwise (floors and the GPU HOT ratchet win over
   the slew, as today).
6. **Actuator mismatch hold (carried §2.9).** While `actuator == Mismatch` both candidates are
   frozen (no integration, no shadow move, no tracking) and `hold: ActuatorMismatch`; the first
   `Verified` read-back unfreezes and resyncs the error. `Unverifiable` (GPU util under the
   verifier floor) is not a hold.
7. **Hold states** reported: `None`, `Parked` (thermal parked, shadow owns the cap),
   `Shadow` (thermal active but shadow selected), `Clamp(Bound::Floor | Bound::Max)` (thermal
   selected at a clamp; the bound identity is carried so the T\* source can read it),
   `ActuatorMismatch`, `GroupUnavailable`, `DrawUnavailable`.
8. **GroupUnavailable dwell.** A group that stays `None` for `GROUP_UNAVAILABLE_DWELL_S = 60`
   while the EC as a whole is valid releases that device to stock (cap = `max`, no lock written
   for an unpowered dGPU) and raises `GroupLost` until the group reads again; the whole-EC
   sensor-lost watchdog is unchanged and still trips the emergency release.

**Write cadence (roast d1).** The cap may change every sample, but the actuator write path
(ryzenadj write + read-back, NVML set + verify, each bounded by `RUN_TIMEOUT`) is synchronous
on the controller thread. A write is issued only when the quantised cap changes **and** at least
`WRITE_MIN_INTERVAL_S = 2` s have passed since that device's last write, except that a floor
move, a guard ratchet, a Released transition and a resume reassert write immediately. The PI and
shadow maths are `t_mono`-driven, so a backlog lags the hardware, not the arithmetic.

Defaults (all `Config` keys, sanitised like every other numeric key, **with positive floors**:
headroom ≥ 1 W / ≥ 30 MHz, fall rate ≥ 0.05 W/s / ≥ 1 MHz/s, band 0.5–10 °C):

| key | CPU | GPU | meaning |
|---|---|---|---|
| `shadow_headroom` | 10 W | 300 MHz | step above draw; the GPU value is also the rising slew per second |
| `shadow_band_c` | 3 °C | 3 °C | below `T* − band` the thermal candidate is parked and the shadow owns the cap |
| `shadow_fall_rate` | 0.33 W/s | 10 MHz/s | one headroom per 30 s (instant when the group is above T\*) |
| `pin_margin` | 1.5 W (`CPU_PINNED_MARGIN_W`, existing) | 30 MHz + util > `GPU_PINNED_UTIL_PCT`, or power-limited (existing values) | "using its cap" |
| `gpu_shadow_enabled` | — | true | off = GPU shadow candidate parked at `max` (spike: measure the watt-hour claim on hardware) |

A load jump from 38 → 100 W under a 54 W `cpu_max_w` therefore ramps to the cap in ~2 s (one 10 W
step per second while pinned and cool); a GPU recovery of 1000 MHz takes ~3.5 s at 300 MHz/s;
a scene dip lets the shadow fall by 10 W in 30 s. The rise time is the tunable that matters:
≳15 s is the onset starvation the previous §2.4 forbade.

**Gains.** `Gains { kc, ti_s }` per device, fitted (§2.6) or defaulted. Defaults: IMC with
λ = max(90, 3·θ_eff), Kc = τ/(K·(λ+θ_eff)), Ti = τ, with θ_eff = EC lag + `ma_interval`/2
for the CPU group and + 40 s more for the GPU group (the `gpu_vr` tail). Units: CPU W/°C, GPU
MHz/°C. Numeric defaults: **CPU** τ = 35 s, K = 0.8 °C/W, θ_eff = 50 s, λ = 150 →
Kc = 0.22 W/°C, Ti = 35 s; **GPU** τ = 35 s, K ≈ 0.02 °C/MHz (0.05 W/MHz from the September
sweep × **0.4 °C/W**, the GPU path's own thermal resistance from the gpu-burn fact — idle die
42 °C → 82 °C at 100 W — not the CPU's 0.8), θ_eff = 90 s, λ = 270 → Kc = 35/(0.02 × 360) ≈
4.9 MHz/°C, Ti = 35 s. Above the card's power-limit knee (~2143 MHz on this card) the clock→heat
gain is ~0; the shadow's draw-tracking (a power-limited card is "pinned" at `draw + headroom`,
not at the lock) bounds the effective ceiling there, and the §4 plant models the plateau so the
sims can see it. Both gain sets are `Config` keys (`cpu_gains`, `gpu_gains`) so a field session
can override them without a fit.

**owns:** `DeviceLoop`, `DeviceDecision`, `Hold`, `Selected`, `Gains`, the shadow-cap
defaults, the write-cadence rule. **consumes:** `t_star` (§2.4), `cpu_group_c`/`gpu_group_c`
(§2.2), the pinned tests (existing `allocator::demand` pinned rules, moved here), the
actuators' clamps and read-back verdicts.

### 2.4 `control/tstar.rs` — `TStarSource` (new, replaces `mode.rs`)

States and transitions:

- **Curve.** Entered when the fw-fanctrl view is fresh, the curve resolves a tread for
  `duty(fan_target)` (today's §2.6 derivation, including the ±snap rule and the duty↔RPM table),
  the replica is reconciled (`EC MISMATCH` clear), **and the EC argmax is a controllable sensor**
  (debounced over `ARGMAX_DEBOUNCE_TICKS` as the prior §2.5/fwloop.10 did — roast d1 Blocking 2),
  held for `ENTRY_HYSTERESIS_S` (15 s at the sample cadence). `T* = min(tread temperature,
  cpu_hot_c − 2, gpu_hot_c − 2)`. Re-derived only when the **curve points or the snapped
  `target_duty` change** (the `points_changed` cache mode.rs keeps today — not on every
  `view_changed`, which fires ~1 sample in 30 in the field), with `resync_error` on both loops
  (no proportional kick).
- **Uncontrollable (new).** Entered from `Curve` or `Held` when the debounced argmax is an
  uncontrollable sensor (ambient, charger): the fan is being set by heat neither loop can touch
  (11–16 % of loaded field samples). T\* is frozen, both thermal candidates are parked so the
  shadows own the caps (devices run at their draw plus headroom), and the `ArgmaxUncontrollable`
  flag is raised. Exit back to `Curve`/`Held` when a controllable sensor takes the argmax
  (debounced). No cap step on either transition (the shadows already hold the applied caps).
- **Held.** Entered from `Curve` when the view goes stale, the curve is invalid, or
  reconciliation fails (§2.6's `EC MISMATCH`), and at Auto entry when no curve is available.
  `T*` starts at the last good curve-derived value (persisted; default 75 °C when none) and is
  driven by the **RPM PI**: `err_rpm = fan_target − fan_smoothed` (`fan_smoothed` = the existing
  `FAN_SMOOTH_N = 5` tail mean of `max(fan1, fan2)`, raw fallback on outage), a slow velocity-form
  PI in °C per RPM at `PI_PERIOD_S = 5`. **Tuning (roast d1):** the inner loops' closed-loop
  constant is λ_inner + θ ≈ 270 + 90 = 360 s; the cascade needs ≥ 4× separation, so
  λ_held = 1440 s → Kc = τ/(K·(λ_held + θ_eff)) = 35/(78 × 1530) ≈ 2.9e-4 °C/RPM at the
  reference slope, Ti = 35 s; output step bounded to 0.5 °C per PI tick. The plant gain
  K ≈ 78 RPM/°C holds at the curve's reference slope only, so the prior design's **slope
  schedule is retained for this loop**: Kc is scaled by `slope_ref / max(slope_at(T*),
  slope_ref)` clamped to `[0.25, 1]`, with the `0.25×` floor when no curve resolves a slope
  (the EC-autofan case has a near-zero slope in 67–73 °C and 140–420 RPM/°C below 64 °C). Output
  clamped to `[T*_floor, min(cpu_hot_c − 2, gpu_hot_c − 2)]` where `T*_floor` = max(uncontrollable
  sensors) + 5 °C (§2.7's feasibility bound), with directional conditional integration at both
  clamps. **Anti-windup:** the integrator holds when raising T\* could not move anything —
  i.e. when both device loops report a hold in `{Clamp(Max), Clamp(Floor), Parked, Shadow,
  GroupUnavailable, DrawUnavailable}` (a device at its shadow cap will not draw more because T\*
  rose; an unavailable device vacuously has no more room). It reads the **previous tick's** Hold
  values (tstar.tick runs before the device ticks).
- **Released.** Fan invalid, EC invalid, watchdog trip, calibration in progress: caps released
  to stock, both loops' integrators seeded on re-entry exactly as `handle_mode_transition`
  does today.
- **Feasibility / steepness (§2.7).** `TargetUnreachable (low)` when the tread duty is below
  the curve's minimum; `(high)` when T\* would exceed the guard ceiling. `SteepCurve` stays as
  an informational flag (the slope schedule above is the only consumer). A per-device
  `DeviceUnreachable` flag is raised when a device has sat at `Clamp(Max)` for
  `BOUND_HOLD_S = 60` with its group under T\* (informational — the load cannot reach the
  target) or at `Clamp(Floor)` with its group over T\* (the real one — the target is too cold
  for this load).

`t_star_last_good` is written on Held exit, on Auto exit, and at most every 60 s while dirty —
never per PI tick.

**owns:** `TStarSource`, its states (incl. `Uncontrollable`), the RPM PI and its slope schedule,
`T*` persistence cadence. **consumes:** `FanctrlView`, the duty↔RPM table, the replica's
reconciliation verdict, `EcReading.argmax`/`all` for controllability and feasibility, both
loops' previous-tick `Hold`.

### 2.5 Controller wiring (`control/controller.rs`, change)

`AutoState` holds `tstar: TStarSource`, `cpu: DeviceLoop<W>`, `gpu: DeviceLoop<Mhz>`, the
replica, the steady window and the warm start. `on_auto_sample`:

1. replica tick (§2.2) → `cpu_group_c`, `gpu_group_c`, reconciliation verdict;
2. `tstar.tick(...)` → `t_star`, flags;
3. guards (§2.8): `gpu_share_override` becomes a **GPU max override**: while `GPU HOT` the GPU
   loop's `max` ratchets down by `DOWN_RATE_MHZ = 105` per sample to no lower than `floor`.
   **Recovery is temperature-gated (roast d1):** `max` climbs back only while the die reads
   ≤ `gpu_hot_c − 4` (84 °C), at half the down rate, so the 86/88 exit/enter band cannot become
   a limit cycle; the closed-loop GPU HOT episode sim bounds the post-episode overshoot. A **CPU
   hot guard** mirrors it: `cpu_hot_c = 90` (Tctl, config key, exit 87) ratchets the CPU loop's
   `max` down by `DOWN_RATE_W = 2` per sample to no lower than `cpu_floor_w`, recovery gated at
   ≤ 85 °C at half rate. Both use the same `MaxRatchet` helper. Note the recovery of the applied
   cap after an episode is governed by the thermal PI (the device is at or above T\* when the
   guard clears), not by the ratchet — the ratchet only restores the ceiling;
4. `cpu.tick(...)`, `gpu.tick(...)` → two caps;
5. write through the existing actuator paths under the §2.3 write-cadence rule:
   `cpu.set_sustained_mw` with read-back (§2.9), `gpu.set_max_clock` with `verify_lock`; the
   read-back verdict is fed back into the next tick as `ActuatorState` (§2.3 step 6); the
   stickiness watchdog, the Mismatch re-write, the shutdown fences and the reassert paths are
   unchanged. **`verify_lock` is re-scoped (roast d1):** because the lock now moves every sample
   under the shadow or a ratchet, the verifier checks `reported ≤ commanded + slack` with a
   strike streak that survives lock changes instead of a per-value state machine that resets on
   every change;
6. warm start: when both groups have sat within 1 °C of T\* and the fan within the steady
   window's tolerance for `STEADY_WINDOW_N` samples, record `(cpu_cap, gpu_lock)` under
   `WarmStart::key(strategy, duty, on_ac)` and refine the duty↔RPM table as today.

**Auto no longer requires calibration** (the `NOT CALIBRATED` gate on `SetAuto(true)` in
`controller.rs` is removed by the controller bead; the flag stays informational, raised when
either device has no fitted gains).

**Auto entry is bumpless by construction:** `draw` is taken as the mean of the last
`SEED_WINDOW_N = 5` valid samples (never one sample; entry is deferred while `draw` is `None`).
Each loop seeds its shadow at `draw + headroom` and its thermal integrator at `max(floor, draw)`.
A cool device (err > band) has its thermal candidate parked, so the first applied cap is the
shadow — `draw + headroom`; a device inside the band applies `min(draw, draw + headroom) = draw`,
i.e. exactly its running state, which the thermal PI then moves at its own pace. With a
warm-start entry the thermal integrator seeds there and the shadow still starts from draw; the
selector picks the min, so a stale warm start can only under-cap by the amount the shadow
allows to ramp back in seconds.

Manual mode (`c`/`g` keys), Monitor, calibration, `p` release, quit and the emergency release
are unchanged in behaviour; they now address the two loops instead of the budget.

### 2.6 Calibration (`calib/`, change to §3.3)

`lut_sweep.rs` is deleted. `step.rs` becomes a per-device step test, run twice by the runner:

1. **Settle** (shared): both devices held at their current applied caps (the loops are frozen,
   `Released`-style, for the calibration's duration as today); the argmax controllable; both
   groups flat within 0.5 °C over 60 s; fans flat within **150 RPM** over 20 s (was 100 — field
   noise at ~2400 RPM spans 100–200); cap **600 s** (was 300). On timeout the skip reason
   names which condition failed and for how long, so the operator can tell "load moved" from
   "fans noisy". **Over-temperature aborts (roast d1 Blocking 3, carried from step.rs):** at any
   point in the run, EC max ≥ `EC_MAX_ABORT_C = 95`, the GPU HOT guard (die ≥ `gpu_hot_c`) or the
   CPU hot guard (Tctl ≥ `cpu_hot_c`) aborts the step — the actuators are restored to their
   pre-step caps and the run ends with a `Noted` reason; the guards have precedence over the
   step writer on every calibration sample.
2. **CPU step:** raise the CPU cap by `STEP_W = 15` W (from the settled cap, clamped to
   `cpu_max_w`), hold the GPU lock; record the CPU group for a **fit window of
   `FIT_WINDOW_S = 360` s** (θ_eff 90 + 5τ 175 + margin — the same window for both devices,
   sized for the slower one), then `fit_fopdt` + `derive_gains` as §3.3, in W/°C, with the
   fitted θ̂ used as-is (no `+ ma_interval/2`, as §3.3 already requires). **Cross-term rejection:** if the GPU group moved more than
   `max(1 °C, 0.2 × ΔT_primary)` during the step window — the modelled coupling of 0.1 °C/°C
   times the primary response, with margin — the fit is rejected ("load changed"), not the
   step.
3. **GPU step:** raise the GPU lock by `STEP_MHZ = 500` (clamped to 3090), hold the CPU cap;
   fit against the GPU group in MHz/°C; reject if the CPU group moved more than
   `max(1 °C, 0.2 × ΔT_primary)`.
4. Save `cpu_gains` / `gpu_gains` **keyed by `(strategy, ma_interval)`** — the step runs with
   the fan curve live, so K and τ embed that curve's rejection; a strategy or interval change
   falls back to defaults until re-fitted. A rejected fit leaves that device on defaults and
   says so.

A step on a device whose group cannot respond (dGPU unpowered, a light load that never heats)
produces a sub-threshold response and is rejected by the existing magnitude rule; the runner
reports it and moves on. `calibrated_at` stamps the run; `NOT CALIBRATED` now means "no fitted
gains for at least one device" and is informational — **Auto no longer requires calibration**
(defaults are safe by construction of the IMC bound).

### 2.7 Guards, watchdog, verification (§2.8, §2.9, unchanged)

GPU hot guard 88 / exit 86 (ratchets the GPU loop's `max`, temperature-gated recovery per
§2.5), the new CPU hot guard 90 / exit 87 (same helper, the CPU loop's `max`), NVMe guard
reporting-only,
`GPU_TRIP_C` 91 and the sensor-lost watchdog (emergency release, latched), CPU read-back
verification with `RUN_TIMEOUT`, GPU `verify_lock` with its `Unverifiable` leg, the stock
restore with read-back and retry, the controller stop fence and bounded joins — all unchanged.

### 2.8 Persistence (`state.rs`, change)

`PersistedState`: `lut` **removed**; `loop_gains` → `cpu_gains` / `gpu_gains:
BTreeMap<String, Gains>` keyed by `"<strategy>:<ma_interval>"` (§2.6); `warm_start: BTreeMap<String, WarmStartEntry { cpu_cap_w, gpu_lock_mhz }>`;
`t_star_last_good: Option<f64>` (new, for `Held`). `validated()` extends to the new fields
(finite, positive, in range). **Migration:** an old file loads with `lut` ignored (one log
line), `loop_gains` ignored (it was Mode A/B gains for one plant — not reusable), and any
`warm_start` value that is a bare number dropped (one log line). No `.bak` is written (carried
nit; out of scope). `Config` gains the shadow-cap keys (with the positive sanitiser floors of §2.3),
`gpu_shadow_enabled`, `cpu_hot_c` (default 90, sanitised strictly below `CPU_TRIP_C` 95 with the
exit margin), and per-device gain overrides. `CPU_MAX_W` / `CPU_MAX_W_FLOOR` move from
`allocator.rs` into `config.rs` **before** the deletion sweep (roast d1) so `cpu_max_w` keeps its
bound. The GPU
bound becomes `gpu_max_mhz` (default 3090, sanitised to `[gpu_floor_mhz, 3090]`); `gpu_max_w`
and `cpu_max_w` stay only as the CPU bound (`cpu_max_w`) — `gpu_max_w` is removed, and an old
config carrying it loads with one warning line.

### 2.9 Telemetry and TUI (§3.5, change)

`decision` line (schema bump to v3): `t_star`, `tstar_state`, per device `{group_c, err_c,
thermal, shadow, cap, selected, hold}`, `cpu_limit_w`, `gpu_max_mhz`, flags. The `budget_w`,
`pi_target_w`, `alloc_*`, `demand_*`, `freeze` columns go. `sample` line unchanged except
`cpu_group_c`/`gpu_group_c` added. TUI: T\* and its state, both group temperatures with their
error, both caps with the binding candidate (`T`/`S`/`F`/`M`), the hold state, the per-device
unreachable flags. Keys unchanged.

## 3. Deletions (sweep source and non-source: path AND basename)

`src/control/budget.rs`, `src/control/allocator.rs` (the pinned tests move to
`device_loop.rs` first), `src/control/spike_antiwindup.rs`, `src/control/mode.rs` (replaced by
`tstar.rs`; the §2.6/§2.7 logic moves, not the file), `src/control/lut.rs`,
`src/calib/lut_sweep.rs`, the watts inner loop in `src/control/gpu_pid.rs` (the rate limiter and
directional conditional integration move into `DeviceLoop`), `DEMAND_MARGIN_W_*`,
`Freeze` (its `ActuatorMismatch` semantics move into `DeviceLoop` step 6), `Budget*` telemetry
fields, `Config::gpu_max_w`, `gpu_share_override`, `PersistedState::lut` and `loop_gains`
(`CPU_MAX_W`/`CPU_MAX_W_FLOOR` are rehomed in `config.rs` first), the `gpu_watts_lut` test fixtures and the `ClockWattsLut` sims, the design
doc's §2.4/§2.5 (marked superseded with a pointer here), and every README/`docs/` mention of
budget, split, LUT, Mode A/Mode B.

## 4. Testing

- **Unit.** `DeviceLoop` as a pure function: thermal PI reaches setpoint on a first-order plant
  within 1 % with ≤ 5 % overshoot at defaults; selector picks the min and tracking keeps the
  unselected candidate at the applied cap (no windup either way, verified by the descent-from-
  max test: after 300 s with the shadow binding, the thermal candidate is within one integral
  step of the cap); shadow rises one headroom per sample only while pinned and cool, falls at
  `fall_rate`, never below floor; hold states as enumerated; the GPU `sat_dir` unwind test
  carried over. `TStarSource`: each transition, hysteresis at 1 Hz, the RPM PI's clamp and hold
  rules, feasibility bounds, persistence of the last good T\*. `EcReading` groups: max over the
  labels, `None` on an unpowered dGPU, the fixture `cros_ec_dgpu_on` re-captured so a `gpu_*`
  sensor is the argmax under load.
- **Simulation** (`ChainedPlant` grows a second thermal node: CPU heat → CPU group with
  τ≈35 s at 0.8 °C/W, GPU clock → GPU group with the `gpu_vr` tail at **0.4 °C/W** (§2.3),
  cross-coupling 0.1 °C/°C each way; a scriptable GPU die temperature (the NVML reading the
  GPU HOT guard keys on) and CPU Tctl so guard episodes can be driven; fault injection for dGPU
  off, a single group's labels dropping out, a stuck-high sensor, fan outage, EC invalid / stale
  view).
  **GPU clock→draw model (no LUT):** `draw_w = load_level × P_full(clock)`, where
  `P_full(clock)` is the September full-load sweep as a piecewise-linear table —
  (1197 MHz, 49.3 W), (1402, 53.5), (1612, 64.2), (1807, 75.9), (1995, 90.8), (2143, 99.4),
  extended flat to 3090 at 100 W (the card's power limit) and linearly to (1000, 45) below —
  and `load_level ∈ [0, 1]` is the scripted GPU load; the reported SM clock is
  `min(lock, clock_at_power_limit(load_level))` — above the knee a loaded card runs BELOW the
  lock, as on hardware (lock 3090, card at ~2520 MHz), which is why the pinned test also counts
  a power-limited card as pinned (§2.3 step 3);
  GPU heat = `draw_w × 0.8 °C/W` into the GPU node. CPU: `draw_w = min(cap, cpu_load_w)` with
  heat `× 0.8 °C/W` into the CPU node, as today.
  1. CPU-heavy, light GPU: CPU group settles at T\*, GPU sits at its shadow cap above draw,
     fans ±150 RPM of target ≥ 90 % of a 30-min converged window.
  2. GPU-heavy, light CPU: the mirror.
  3. Both heavy: both groups at T\*, same fan bar.
  4. Load step light → heavy on the GPU: cap ramps at the rising slew (1000 MHz in ≤ 5 s),
     no fan crest above target+250, **the GPU group's temperature overshoots T\* by ≤ 2 °C and
     settles within one λ**, the CPU cap unchanged during the step.
  5. Auto entry under a steady heavy load: neither device's draw changes by more than its
     `pin_margin` in the first 10 s; fans do not fall.
  6. Curve loss mid-session from a converged state: `Held` entered, T\* moves from the last
     good value, fans return within ±150 RPM within 20 min (λ_held = 1440 s) with a cap-trace
     hunting check at the ~1000 s period; curve return → `Curve` with no step in either cap.
  7. Unreachable device: a GPU that cannot reach T\* never changes the CPU cap.
  8. Configuration smoke: each `TStarSource` state (incl. `Uncontrollable`, via an
     ambient-dominated argmax leg) and each `Hold` / `Selected` value is reached at least once
     across the sims (the checklist is behavioural, not self-pushed vectors).
  9. GPU HOT episode: a 5-min die-temperature excursion above 88 °C — the ratchet reaches the
     floor, recovery starts only below 84 °C, no re-trip within the episode's tail, post-episode
     fan overshoot ≤ 150 RPM (the prior design's bar) and the cap back within 10 % of its
     pre-episode value within 5 min.
  10. Robustness: sims 1–3 repeated with the plant's K, τ and θ each perturbed by ±50 %, and a
      period-agnostic **no-relay rule** on every 30-min window (no sustained oscillation of the
      cap or the fan with a peak-to-peak above 100 RPM / 4 W / 150 MHz at any period from 30 s
      to 20 min) — a loop hunting slowly inside ±150 RPM does not pass.
  11. Stuck-high sensor: one GPU-group label pinned at 105 °C for 5 min raises `EcImplausible`
      and the GPU cap does not fall to the floor.
- **Hardware (the user's, parked):** the 30-min gaming check; the VR/VRAM label spike.

## 5. Follow-ons (not in this tree)

- `state.json.bak` on migration; the telemetry `loop_mode` column and `Decision.reasons`
  consumer from the earlier roast nit list; the Instant-vs-suspend clock escalation (design doc
  §270 vs `sampler.rs`), still open.
- The upstream fw-fanctrl `movingAverageInterval` change (60 → 20 s) as a way to shorten the
  dead time — an operator config decision, not code.

## Post-Implementation Notes

*As this design is implemented and iterated on — bug fixes, adjustments, anything that diverged from the assumptions above — append a dated note here, whether or not a formal debugging skill was used.*

# Per-device temperature loops

Date: 2026-09-11 · Status: draft · Supersedes §2.4 and §2.5 of
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
   than one boosting to 3 GHz at 56 % — the GPU shadow cap binds continuously and pays for
   itself in watt-hours; on the CPU `ryzenadj` caps average power, so the win is boost-peak
   clipping only.
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
positive readings of each group's labels, `None` when the group has no positive reading (dGPU
unpowered → GPU group `None`). Group membership is a fixed label list in `ec.rs` (the table
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
`t_star`, `group_c: Option<f64>`, `draw` (CPU: `cpu_pkg_w`; GPU: `gpu_sm_mhz` with
`gpu_util_pct`), `floor`, `max`, `pinned_test` result. Output: the cap to apply and a
`DeviceDecision` record for telemetry.

Per tick:

1. `err = t_star − group_c`. If `group_c` is `None` the loop returns the last applied cap
   unchanged and marks `hold: GroupUnavailable`.
2. **Thermal candidate.** Velocity-form PI on `err` with the device's `Gains { kc, ti_s }`,
   `PI_PERIOD_S = 5` as today (the loop runs its integrator every 5th sample; the shadow cap
   and selector run every sample). Output clamped to `[floor, max]`. The integrator is
   updated only when `selected == Thermal` on the previous PI tick **and** the error does not
   push further into an active output clamp (directional conditional integration — the
   `sat_dir` rule from `gpu_pid.rs`). Otherwise the integrator holds its last value.
3. **Shadow candidate.** `shadow = draw + headroom`, then:
   - if `pinned` (draw within `pin_margin` of the applied cap) **and** `group_c < t_star −
     band`: `shadow_target = applied + headroom` — one headroom step up this sample;
   - else `shadow_target = draw + headroom`;
   - `shadow` moves toward `shadow_target` at once when rising, and at `fall_rate` (units per
     second) when falling.
   Clamped to `[floor, max]`.
4. **Selector.** `cap = min(thermal, shadow)`; `selected ∈ {Thermal, Shadow, Floor, Max}` names
   which bound produced it. **Tracking:** the candidate not selected has its state set to
   `cap` (the thermal integrator's `u`/`v`; the shadow's current value), so both start the next
   tick from the applied value and neither winds while the other binds.
5. **Quantise and slew** as the allocator does today: 0.5 W grid on the CPU, the actuator's
   105 MHz/s rate limit on the GPU (floors win over the rate limit, as today).
6. **Hold states** reported: `None`, `Shadow` (thermal held because shadow selected),
   `Clamp` (thermal held at floor/max), `GroupUnavailable`.

Defaults (all `Config` keys, sanitised like every other numeric key):

| key | CPU | GPU | meaning |
|---|---|---|---|
| `shadow_headroom` | 10 W | 300 MHz | step above draw |
| `shadow_band_c` | 3 °C | 3 °C | how far below T\* a pinned device may raise its shadow |
| `shadow_fall_rate` | 0.33 W/s | 10 MHz/s | one headroom per 30 s |
| `pin_margin` | 2 W (existing `CPU_PINNED_MARGIN_W`) | 105 MHz + util > existing `GPU_PINNED_UTIL_PCT` | "using its cap" |

A load jump from 38 → 100 W therefore ramps in about 6 s (one 10 W step per second while the
device stays pinned and cool); a scene dip lets the shadow fall by 10 W in 30 s. The rise time
is the tunable that matters: ≳15 s is the onset starvation the previous §2.4 forbade.

**Gains.** `Gains { kc, ti_s }` per device, fitted (§2.6) or defaulted. Defaults: IMC with
λ = max(90, 3·θ_eff), Kc = τ/(K·(λ+θ_eff)), Ti = τ, with θ_eff = EC lag + `ma_interval`/2
for the CPU group and + 40 s more for the GPU group (the `gpu_vr` tail). Units: CPU W/°C, GPU
MHz/°C. Numeric defaults: **CPU** τ = 35 s, K = 0.8 °C/W, θ_eff = 50 s, λ = 150 →
Kc = 0.22 W/°C, Ti = 35 s (the previous single-loop defaults, whose plant was CPU-dominated);
**GPU** τ = 35 s, K ≈ 0.04 °C/MHz (0.05 W/MHz from the September LUT × 0.8 °C/W), θ_eff = 90 s
(50 s + the 40 s `gpu_vr` tail), λ = 270 → Kc = 35/(0.04 × 360) ≈ 2.4 MHz/°C, Ti = 35 s. Both
are `Config` keys (`cpu_gains`, `gpu_gains`) so a field session can override them without a
fit.

**owns:** `DeviceLoop`, `DeviceDecision`, `Hold`, `Selected`, `Gains`, the shadow-cap
defaults. **consumes:** `t_star` (§2.4), `cpu_group_c`/`gpu_group_c` (§2.2), the pinned tests
(existing `allocator::demand` pinned rules, moved here), the actuators' clamps.

### 2.4 `control/tstar.rs` — `TStarSource` (new, replaces `mode.rs`)

States and transitions:

- **Curve.** Entered when the fw-fanctrl view is fresh, the curve resolves a tread for
  `duty(fan_target)` (today's §2.6 derivation, including the ±snap rule and the duty↔RPM table),
  and the replica is reconciled (`EC MISMATCH` clear), held for `ENTRY_HYSTERESIS_S` (15 s at
  the sample cadence, as fixed on 2026-09-10). `T* = tread temperature`. Re-derived on every
  `view_changed` and fan-target change, with `resync_error` on both loops (no proportional
  kick), exactly as §2.4 required for the single loop.
- **Held.** Entered from `Curve` when the view goes stale, the curve is invalid, or
  reconciliation fails (§2.6's `EC MISMATCH`), and at Auto entry when no curve is available.
  `T*` starts at the last good curve-derived value (persisted; default 75 °C when none) and is
  driven by the **RPM PI**: `err_rpm = fan_target − fan_smoothed`, a slow velocity-form PI
  (Kc in °C per RPM, Ti several minutes — defaults derived so its closed-loop constant is ≥ 3×
  the slower device loop's) whose output is clamped to `[T*_floor, gpu_hot_c − 2]` where
  `T*_floor` = max(uncontrollable sensors) + 5 °C (§2.7's feasibility bound). Its integrator
  holds when both device loops report `Hold::Clamp` at `max` (nothing left to heat) or at
  `floor` (nothing left to cool) — the same "would the extra move anywhere" rule the device
  loops use.
- **Released.** Fan invalid, EC invalid, watchdog trip, calibration in progress: caps released
  to stock, both loops' integrators seeded on re-entry exactly as `handle_mode_transition`
  does today.
- **Feasibility / steepness (§2.7).** `TargetUnreachable (low)` when the tread duty is below
  the curve's minimum; `(high)` when T\* would exceed the guard ceiling. `SteepCurve` stays as
  an informational flag (no gain schedule remains). A per-device `DeviceUnreachable` flag is
  raised when a device has sat at `max` for `BOUND_HOLD` with its group under T\* (informational
  — the load cannot reach the target) or at `floor` with its group over T\* (the real one — the
  target is too cold for this load).

**owns:** `TStarSource`, its states, the RPM PI, `T*` persistence. **consumes:** `FanctrlView`,
the duty↔RPM table, the replica's reconciliation verdict, `EcReading.all` for feasibility, both
loops' `Hold`.

### 2.5 Controller wiring (`control/controller.rs`, change)

`AutoState` holds `tstar: TStarSource`, `cpu: DeviceLoop<W>`, `gpu: DeviceLoop<Mhz>`, the
replica, the steady window and the warm start. `on_auto_sample`:

1. replica tick (§2.2) → `cpu_group_c`, `gpu_group_c`, reconciliation verdict;
2. `tstar.tick(...)` → `t_star`, flags;
3. guards (§2.8): `gpu_share_override` becomes a **GPU max override**: while `GPU HOT` the GPU
   loop's `max` ratchets down by `DOWN_RATE` per tick to no lower than `floor`, exactly the
   current share ratchet expressed in MHz;
4. `cpu.tick(...)`, `gpu.tick(...)` → two caps;
5. write through the existing actuator paths: `cpu.set_sustained_mw` with read-back (§2.9),
   `gpu.set_max_clock` with `verify_lock`; the stickiness watchdog, the Mismatch re-write,
   the shutdown fences and the reassert paths are unchanged;
6. warm start: when both groups have sat within 1 °C of T\* and the fan within the steady
   window's tolerance for `STEADY_WINDOW_N` samples, record `(cpu_cap, gpu_lock)` under
   `WarmStart::key(strategy, duty, on_ac)` and refine the duty↔RPM table as today.

**Auto entry is bumpless by construction:** with no warm-start entry each loop seeds its
thermal integrator at `max(floor, draw)` — the CPU at its current package watts, the GPU at its
current SM clock — and its shadow at `draw + headroom`, so the first applied cap equals the
running state plus headroom. With a warm-start entry the thermal integrator seeds there and the
shadow still starts from draw; the selector picks the min, so a stale warm start can only
under-cap by the amount the shadow allows to ramp back in seconds.

Manual mode (`c`/`g` keys), Monitor, calibration, `p` release, quit and the emergency release
are unchanged in behaviour; they now address the two loops instead of the budget.

### 2.6 Calibration (`calib/`, change to §3.3)

`lut_sweep.rs` is deleted. `step.rs` becomes a per-device step test, run twice by the runner:

1. **Settle** (shared): both devices held at their current applied caps (the loops are frozen,
   `Released`-style, for the calibration's duration as today); the argmax controllable; both
   groups flat within 0.5 °C over 60 s; fans flat within **150 RPM** over 20 s (was 100 — field
   noise at ~2400 RPM spans 100–200); cap **600 s** (was 300). On timeout the skip reason
   names which condition failed and for how long, so the operator can tell "load moved" from
   "fans noisy".
2. **CPU step:** raise the CPU cap by `STEP_W = 15` W (from the settled cap, clamped to
   `cpu_max_w`), hold the GPU lock; record the CPU group for the fit window; `fit_fopdt` +
   `derive_gains` as §3.3, in W/°C. **Cross-term rejection:** if the GPU group moved more than
   1 °C during the step window the fit is rejected ("load changed"), not the step.
3. **GPU step:** raise the GPU lock by `STEP_MHZ = 500` (clamped to 3090), hold the CPU cap;
   fit against the GPU group in MHz/°C; reject if the CPU group moved more than 1 °C.
4. Save `cpu_gains` / `gpu_gains`; a rejected fit leaves that device on defaults and says so.

A step on a device whose group cannot respond (dGPU unpowered, a light load that never heats)
produces a sub-threshold response and is rejected by the existing magnitude rule; the runner
reports it and moves on. `calibrated_at` stamps the run; `NOT CALIBRATED` now means "no fitted
gains for at least one device" and is informational — **Auto no longer requires calibration**
(defaults are safe by construction of the IMC bound).

### 2.7 Guards, watchdog, verification (§2.8, §2.9, unchanged)

GPU hot guard 88 / exit 86 (ratchets the GPU loop's `max`), NVMe guard reporting-only,
`GPU_TRIP_C` 91 and the sensor-lost watchdog (emergency release, latched), CPU read-back
verification with `RUN_TIMEOUT`, GPU `verify_lock` with its `Unverifiable` leg, the stock
restore with read-back and retry, the controller stop fence and bounded joins — all unchanged.

### 2.8 Persistence (`state.rs`, change)

`PersistedState`: `lut` **removed**; `loop_gains` → `cpu_gains: Option<Gains>`,
`gpu_gains: Option<Gains>`; `warm_start: BTreeMap<String, WarmStartEntry { cpu_cap_w, gpu_lock_mhz }>`;
`t_star_last_good: Option<f64>` (new, for `Held`). `validated()` extends to the new fields
(finite, positive, in range). **Migration:** an old file loads with `lut` ignored (one log
line), `loop_gains` ignored (it was Mode A/B gains for one plant — not reusable), and any
`warm_start` value that is a bare number dropped (one log line). No `.bak` is written (carried
nit; out of scope). `Config` gains the shadow-cap keys and per-device gain overrides. The GPU
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
`Freeze::DemandLimited`, `Budget*` telemetry fields, `Config::gpu_max_w`, `PersistedState::lut`
and `loop_gains`, the `gpu_watts_lut` test fixtures and the `ClockWattsLut` sims, the design
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
  τ≈35 s, GPU clock → GPU group with the `gpu_vr` tail, cross-coupling 0.1 °C/°C each way):
  1. CPU-heavy, light GPU: CPU group settles at T\*, GPU sits at its shadow cap above draw,
     fans ±150 RPM of target ≥ 90 % of a 30-min converged window.
  2. GPU-heavy, light CPU: the mirror.
  3. Both heavy: both groups at T\*, same fan bar.
  4. Load step light → heavy on the GPU: cap ramps over ~6 s, no fan crest above target+250,
     the CPU cap unchanged during the step.
  5. Auto entry under a steady heavy load: neither device's draw changes by more than its
     `pin_margin` in the first 10 s; fans do not fall.
  6. Curve loss mid-session: `Held` entered, T\* moves from the last good value, fans return
     within ±150 RPM within 10 min; curve return → `Curve` with no step in either cap.
  7. Unreachable device: a GPU that cannot reach T\* never changes the CPU cap.
  8. Configuration smoke: each `TStarSource` state and each `Hold` value is reached at least
     once across the sims (the checklist is behavioural, not self-pushed vectors).
- **Hardware (the user's, parked):** the 30-min gaming check; the VR/VRAM label spike.

## 5. Follow-ons (not in this tree)

- `state.json.bak` on migration; the telemetry `loop_mode` column and `Decision.reasons`
  consumer from the earlier roast nit list; the Instant-vs-suspend clock escalation (design doc
  §270 vs `sampler.rs`), still open.
- The upstream fw-fanctrl `movingAverageInterval` change (60 → 20 s) as a way to shorten the
  dead time — an operator config decision, not code.

## Post-Implementation Notes

*As this design is implemented and iterated on — bug fixes, adjustments, anything that diverged from the assumptions above — append a dated note here, whether or not a formal debugging skill was used.*

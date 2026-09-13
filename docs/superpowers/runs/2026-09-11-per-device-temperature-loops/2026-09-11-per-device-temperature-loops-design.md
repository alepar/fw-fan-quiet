# Per-device temperature loops

Date: 2026-09-11 · Status: draft (revision 4, after design roast iteration 3) · Supersedes §2.4 and §2.5 of
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
   candidate; the applied cap is the min of the thermal and shadow candidates. A one-time
   handover on entry to a hot episode (§2.3 step 4) starts thermal regulation at the last
   applied cap; draw never continuously overwrites the PI state. A GPU clock lock is a frequency
   ceiling, so a card locked at 2 GHz running 80 % duty does the same work at a lower V/F point
   than one boosting to 3 GHz at 56 % — the GPU shadow cap binds continuously and is expected to pay for
   itself in watt-hours — **an unmeasured claim** (roast d1): it is inert while the card is
   power-limited, and the field replays show the card mostly below the knee; `gpu_shadow_enabled`
   exists so the user can A/B it during the hardware check (off = thermal-only GPU control,
   §2.3 defaults table). On the CPU `ryzenadj` caps average
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
   ├─ Uncontrollable: ambient/charger holds the argmax → T* frozen, loops in Bypass (shadows own the caps)
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
(dGPU unpowered → GPU group `None`). A control reading must be finite and satisfy
`0 < value <= EC_PLAUSIBLE_MAX_C = 110`; otherwise it is omitted from `max_c`, `argmax`,
`all`, the group maxima and T* feasibility inputs, with `EcImplausible` naming the label.
These are control-facing fields. A separate `reconciliation_max_c: Option<i32>` uses every
finite positive reading, rounded and maximised exactly as the carried replica does, **including
values above 110 °C**. Only this stream feeds the argmax reconciliation boxcar (§2.2), matching
fw-fanctrl's positive-only filter. It never feeds a device error, T* feasibility or argmax
controllability. Thus a 150 °C fault cannot manufacture an `EC MISMATCH` solely because our
control gate rejected it; genuine timing/history mismatches still score normally. `ec_valid`
continues to require at least one plausible control reading, so a raw-only stream cannot keep
an otherwise-invalid EC in Auto.

There is **no jump rule**. A plausible stuck device-group reading (e.g. 105 °C from startup)
stays in that group's max; cooling toward the device floor is the accepted safe direction and
`DeviceUnreachable` reports it after `BOUND_HOLD_S`. This argument applies to device groups;
a stuck uncontrollable label has the separate visible backstop in §2.4.

Membership uses the exact table above first, then the carried prefix fallback (`cpu` / `apu`
→ CPU, `gpu_` → GPU). Every unexpected label or missing expected label raises informational
`EcUnknownLabel` naming it, at discovery and on subsequent label-set changes; log only when
that diagnostic set changes. A present label with a sentinel or ENODATA is not a missing label.
Fallback labels remain in their device group's max. Only the two exact ambient/charger labels
are known uncontrollable sensors; an unmatched label with no device prefix is **Unknown**,
not ambient/charger. It remains visible in the plausible `all`/`argmax` and raw reconciliation
streams but cannot enter `Uncontrollable`. An Unknown argmax forces `Held`, inhibits `Curve`
entry and raises `EcUnknownLabel`; if no device group is available, the per-device unavailable
rules apply. `is_controllable()` means CPU or GPU membership, while `is_uncontrollable()`
explicitly means one of the known ambient/charger labels; Unknown is neither.

**owns:** group classification, label diagnostics, plausible control fields and the separate
positive-only reconciliation maximum. **consumes:** nothing new.

### 2.2 `EcReplica` — per-group boxcar (change to §2.2)

The replica has three boxcars over the same `ma_interval`, capped at 100 samples and retaining
the carried mean-before-append off-by-one. The reconciliation boxcar consumes
`reconciliation_max_c` and is compared with fw-fanctrl's `movingAverageTemperature`; CPU and
GPU boxcars consume only their own plausible instantaneous group maxima.

On Auto engagement, re-engagement from Released, calibration exit, resume, and an explicit
reseed after `EC MISMATCH` clears, discard the old histories. Seed the reconciliation boxcar
from a fresh `view.ma_temperature` as in the carried §2.2; without a usable view it starts
unseeded and accumulates its own raw maximum stream, with reconciliation remaining
unreconciled until a usable full window and successful scored comparison. Seed **each group
independently with N copies of its own current instantaneous group max**, making that group's
first reported average exactly its own value. Never use the socket's single argmax average as
a group seed. A missing group instead clears its own boxcar and reports `None`; the first
plausible sample on its return seeds that group in the same way before ordinary averaging.
The corresponding device cap follows §2.3's unavailable/recovery rules.

`view_changed` is **not a reseed or invalidation event**. A changed interval calls
`set_interval` on all three boxcars, retaining existing samples and trimming only excess
oldest samples when N shrinks; when it grows, average the retained history while new samples
fill it. An ordinary poll, strategy change or points edit preserves all histories and seed
state. Reconciliation failure disables Curve through §2.4 but does not itself erase valid
group histories; the explicit mismatch-clear reseed above is the reset event. Resume also
clears the fan/steady windows and elapsed-time baselines as the controller contract requires.

### 2.3 `control/device_loop.rs` — `DeviceLoop` (new)

One struct, instantiated for CPU (units W) and GPU (units MHz). Inputs per 1 Hz tick:
`t_star`, `group_c: Option<f64>`, `draw: Option<unit>` (CPU: the `SEED_WINDOW_N = 5`-sample
tail mean of `cpu_pkg_w` while RAPL has a valid delta — the mean, not one sample, because the
fixed 53 W fast limit lets package power burst above the sustained cap for single samples;
`None` on the first sample and after a counter reset; GPU: `gpu_sm_mhz` when `gpu_mhz_valid`),
`floor`, `max`, `mode: ThermalMode { Regulate | Bypass }` (Bypass = the thermal candidate
reports `max` and its PI does not run — used by `Uncontrollable`, §2.4), and
`actuator: ActuatorState { Verified | Mismatch | Unverifiable }` from the previous write's
read-back. Output: the cap to apply and a `DeviceDecision` record for telemetry.

**Two candidates with a one-time thermal handover (revision 4).** The thermal PI
owns regulation; the shadow shapes cool-device load changes. The previous continuous
`thermal := min(thermal, shadow)` rule is removed: a draw dip is not evidence that a
lower thermal limit is needed. Both shadow directions stop while regulating above T*.
The handover is an edge, not a condition repeated while hot.

Timing input includes `dt_s`, `resumed`, and `delta_tstar` (new minus previous T*). Use
`dt_control = clamp(dt_s, 0, 2)` seconds (non-finite or negative means zero);
accumulate it to a 5 s PI period, run at most once per sample using the accumulated
interval (bounded to 7 s), then clear the accumulator. Shadow slews use `dt_control`.
A resumed sample holds the pre-suspend cap, clears the PI time accumulator and draw
window, reseeds boxcars (§2.2), and resyncs each error; it performs no PI or shadow step.
Elapsed wall time does not count toward hot, missing-data, bound or verification
streaks across resume; restart those dwells. Guards and emergency release still win.

1. `err = t_star − group_c`. A missing group holds the last applied cap (or `max`
   if none), with `GroupUnavailable`; step 8 defines its dwell. Missing draw does
   not stop the thermal PI. It freezes the shadow with `DrawUnavailable`; after
   60 s of valid control time it requests shadow = max, using the normal output
   rise slew, so control becomes thermal-only without an upward step. On draw
   return seed the shadow at the current applied cap, then resume its slews.
2. **Thermal PI.** Velocity-form PI at the elapsed 5 s cadence:
   `u_next = u + Kc*(err-e_prev) + Kc*elapsed_s/Ti*err`.
   Clamp to `[floor,max]`; suppress only an integral term pushing farther into
   an active bound. Update `e_prev` even when integration is suppressed.
   In Regulate the PI runs irrespective of which candidate binds. In Bypass it
   is frozen and reports max. `resync_error` sets the previous error to the
   current error without changing output or applying a proportional kick.
3. **Shadow.** Target = clamp(draw + headroom, floor, max). In Regulate it
   rises at CPU headroom/s or GPU 300 MHz/s and falls at the configured fall
   rate **only when err ≥ 0**; with err < 0 both directions hold. In Bypass
   both directions run regardless of err (the frozen T* is not a gate).
   Missing draw holds it as step 1 specifies. The shadow is never continuously
   tracked to thermal output. Mode transitions seed it explicitly below.
4. **Handover and selector.** Before any candidate motion, detect a measured hot crossing:
   `prev_group <= prev_tstar && group > prev_tstar && group > tstar`,
   with a rearmed hot-episode latch and previous decision selected Shadow.
   On this one tick seed `u = thermal = clamp(last_applied, floor, max)`,
   resync error, and skip the PI increment. Set a `hot_episode` latch.
   No further handover is allowed until err has been nonnegative for 5 s of
   control time. Initial entry with a hot group uses the entry shadow as
   last_applied for this one handover. On a T* change shift `e_prev += delta_tstar` before the PI update:
   this cancels only the setpoint's proportional kick and preserves the
   measured-temperature contribution. Preserve the hot latch and its
   debounce across Held updates. The measured-crossing predicate prevents a target-only error sign change
   from manufacturing a handover. Ordinary PI integration
   still regulates toward the new target. Record previous group and T*
   on every valid tick, including non-PI ticks.
   Select `cap_requested = min(thermal, shadow)`. Bound precedence:
   if requested == floor, Selected=Floor; else if requested == max,
   Selected=Max; otherwise thermal ≤ shadow selects Thermal, else Shadow.
   Clamp comparisons use the unquantised clamped candidates, not telemetry
   rounding. This makes both clamp identities reachable, including ties.
   A hot draw dip leaves shadow unchanged and cannot overwrite thermal;
   ordinary PI movement from the measured temperature remains possible.

   **Mode transfers.** On Bypass entry seed shadow to last_applied, suppress
   candidate motion for that tick, and freeze thermal. On exit seed thermal
   to last_applied, resync error, clear the PI accumulator, suppress motion
   for the transition tick and initialize the hot latch from the new error.
   This removes both entry and exit steps; later Bypass load changes use
   the normal shadow slew. The output slew applies even with shadow disabled.

5. **Quantise and slew.** 0.5 W grid on the CPU. All upward applied-cap motion is bounded by CPU
   headroom/s or GPU 300 MHz/s when Shadow binds, GPU 105 MHz/s otherwise;
   GPU downward motion is bounded by 105 MHz/s. Accumulate this slew from
   the previous slew-limited request even on samples with no actuator write;
   the write path sends the latest request when due. Use dt_control, including
   configuration toggles and missing-draw dwell expiry. Floor changes and
   hot-guard downward ratchets bypass slew. Bounds win after quantisation.
6. **Actuator mismatch hold (carried §2.9).** While `actuator == Mismatch` both candidates are
   frozen (no integration, no shadow move, no handover) and `hold: ActuatorMismatch`; the first
   `Verified` read-back unfreezes and resyncs the error. `Unverifiable` (GPU util under the
   verifier floor) is not a hold. **Guards have precedence over the freeze:** the applied cap
   is always `min(cap, max)`, so a hot-guard ratchet of `max` (§2.5 step 3) lowers even a frozen
   cap, and that change writes immediately under the cadence rule below.
7. **Hold states** reported: `None`, `Shadow` (shadow selected), `Clamp(Bound::Floor |
   Bound::Max)` (requested cap at that bound, including a tie; the bound identity is carried), `ActuatorMismatch`, `GroupUnavailable`, `DrawUnavailable`, `Bypass`.
8. **GroupUnavailable dwell.** A group that was available in this Auto session and then reads
   `None` for `GROUP_UNAVAILABLE_DWELL_S = 60` while the EC as a whole is valid has that device
   **released to `max`** (`cpu_max_w` / `gpu_max_mhz` — not the stock limits; a normal cap
   write) with `GroupLost` raised until the group reads again, after which regulation resumes
   from `max` (thermal at `max`, shadow reseeded at `draw + headroom`). A group that is `None`
   from the first tick (dGPU unpowered) is **absent, not lost**: `hold: GroupUnavailable`, the
   loop reports `max`, the controller skips the actuator write (§2.5), no dwell, no flag; the
   whole-EC sensor-lost watchdog is unchanged and still trips the emergency release.

**Write cadence (roast d1).** The cap may change every sample, but the actuator write path
(ryzenadj write + read-back, NVML set + verify, each bounded by `RUN_TIMEOUT`) is synchronous
on the controller thread. A write is issued only when the quantised cap changes **and** at least
`WRITE_MIN_INTERVAL_S = 2` s have passed since that device's last write, except that a floor
move, a guard ratchet, a Released transition and a resume reassert write immediately. The elapsed-time contract above bounds PI and shadow arithmetic during a backlog.
A pending cap is not evidence of a successful write: last_applied means the last
successfully commanded cap; write cadence may delay a requested change.

Defaults (all `Config` keys, sanitised like every other numeric key, **with positive floors**:
headroom ≥ 1 W / ≥ 30 MHz, fall rate ≥ 0.05 W/s / ≥ 1 MHz/s):

| key | CPU | GPU | meaning |
|---|---|---|---|
| `shadow_headroom` | 10 W | 300 MHz | step above draw; also the rising slew per second (CPU 10 W/s, GPU `GPU_RISE_SLEW`) |
| `shadow_fall_rate` | 0.33 W/s | 10 MHz/s | one headroom per 30 s |
| `gpu_shadow_enabled` | — | true | off = shadow requests `max`; output rise remains slew-limited; thermal-only control has the full max-to-knee dead zone (tested below) |

`shadow_band_c` and the `pin_margin` row of rev2 are gone (no band, no pinned test); the
`CPU_PINNED_MARGIN_W` / `GPU_PINNED_MARGIN_MHZ` / `GPU_PINNED_UTIL_PCT` constants are deleted
with `allocator.rs` (§3).

A load jump from 38 → 100 W under a 54 W `cpu_max_w` therefore ramps to the cap in ~2 s (10 W
per second while the group is at or below T\*). A GPU shadow request slews at 300 MHz/s, but
the measured-clock target exposes one 300 MHz headroom step per successful command; at the
required 2 s write cadence, an applied 1000 MHz recovery therefore takes at most 8 s. A scene
dip lets the shadow fall by 10 W in 30 s; a one-sample 20 W burst on an
unstressed CPU moves the 5-sample draw mean by 4 W, so the shadow rises 4 W and falls back in
12 s — it does not go to `max`. The rise time is the tunable that matters: ≳15 s is the onset
starvation the previous §2.4 forbade.

**The GPU dead zone.** Above the power-limit knee (~2143 MHz at full load)
clock-to-heat gain is approximately zero. With draw available and shadow enabled,
a settled power-limited device has shadow ≤ reported plateau + headroom. A genuine hot
handover starts thermal at that cap. Its zero-gain travel is therefore measured as
`D = max(0, handover_cap − knee)`; it is not a fixed one-headroom bound because the reported
full-load plateau (~2520 MHz in the plant) can itself sit above the knee. At default
Kc/Ti = 1/(0.02*360) ≈ 0.1389 MHz/(°C·s), even 300 MHz costs about 540 s at 4 °C error or
720 s at 3 °C, before the 90 s response delay. The measured crossing interval cannot be
included in a 3λ settling budget.

That 300 MHz bound is conditional: disabled shadow, missing draw after the
dwell, or a setpoint change while thermal is already above the shadow can leave
up to max−knee ≈ 947 MHz (about 2273 s at 3 °C) of travel. These are explicit
thermal-only sim legs, not covered by the normal headroom claim.
For a plateau interval with |err| ≥ e_min > 0 and non-increasing err, the
integral-only upper bound is D/(Kc/Ti*e_min), with D the thermal output's
distance to the knee at that interval's start; the proportional term helps.
The sim records the first downward knee crossing after a hot response
(the lock begins reducing full-load power), asserts this bound plus one PI
period, and measures settling from that crossing plus θ_eff. If the hot
response starts below the knee, D=0 and its start is the crossing time.
An earlier upward crossing during load onset does not start the settling
clock. It does not divide by
zero or assert a finite crossing time when the group never exceeds T*.
The GPU HOT guard remains the independent protection during this travel.

**Gains.** `Gains { kc, ti_s }` per device, fitted (§2.6) or defaulted. Defaults: IMC with
λ = max(90, 3·θ_eff), Kc = τ/(K·(λ+θ_eff)), Ti = τ, with θ_eff = EC lag + `ma_interval`/2
for the CPU group and + 40 s more for the GPU group (the `gpu_vr` tail) — **computed from the
live `ma_interval`** at Auto entry and on an interval change (roast d2 nit), not frozen at the
60 s numbers below. Units: CPU W/°C, GPU MHz/°C. Numeric defaults at `ma_interval` = 60 s:
**CPU** τ = 35 s, K = 0.8 °C/W, θ_eff = 50 s, λ = 150 → Kc = 0.22 W/°C, Ti = 35 s; **GPU**
τ = 15 s (provisional), K ≈ 0.02 °C/MHz (0.05 W/MHz from the September sweep × **0.4 °C/W**, the GPU path's
own thermal resistance from the gpu-burn fact — idle die 42 °C → 82 °C at 100 W — not the CPU's
0.8), θ_eff = 90 s, λ = 270 → Kc = 15/(0.02 × 360) ≈ 2.1 MHz/°C, Ti = 15 s.
The offline replay of both September 9 gpu-burn CSVs gives raw GPU-group
rise fits τ=11.26/13.89 s; individual VR/VRAM poles are about 35–43 s.
Fans changed during those captures and the argmax switched sensors, so 15 s
is an evidence-informed provisional default, not a measured physical pole.
[GPU time-constant evidence](tools/gpu-tau-evidence.md) records source hashes,
fit method, horizon sensitivity and reproducible commands. The offline
acceptance crosses GPU τ={8,15,25,50} s with K={0.01,0.02,0.03} °C/MHz
and θ_eff={45,90,135} s; nominal controller gains stay fixed during those
perturbations. Fitted per-device gains supersede this assumption. **Precedence
(roast d2):** a `Config` override (`cpu_gains` / `gpu_gains`) wins over a fitted entry for the
current `(strategy, ma_interval)` key, which wins over the defaults; the controller resolves
this at Auto entry and on a strategy/interval change and reports the source (`gains_source ∈
{config, fitted, default}` per device on the decision line and in the TUI) so a stale override
is visible, not silent.

**owns:** `DeviceLoop`, `DeviceDecision`, `Hold`, `Selected`, `Gains`, `ThermalMode`, the
shadow-cap defaults, the write-cadence rule. **consumes:** `t_star` (§2.4),
`cpu_group_c`/`gpu_group_c` (§2.2), the actuators' clamps and read-back verdicts.

### 2.4 `control/tstar.rs` — `TStarSource` (new, replaces `mode.rs`)

States and transitions. **Auto entry starts in `Held`**: use `t_star_last_good` only when its
`(strategy, fan_target_rpm)` key matches the current resolved strategy and sanitised requested
fan target, its timestamp is not in the future, and its age is at most
`TSTAR_SEED_MAX_AGE_S = 21600` (6 h). Otherwise seed from the max over the currently available
controllable-group averages, never from a previous strategy or target. Clamp the seed to the
current `[T*_floor, T*_ceiling]`; when neither group is available use `T*_ceiling` and let both
device-unavailable rules apply. A shared max gives zero error only to the hottest group; the
cooler group's error is non-negative before any required feasibility clamp. Resync each loop's
previous error to its actual seeded error to suppress a proportional kick; cap seeding follows
§2.5. The entry hysteresis and argmax debounce then run from there. `Held` means
"T\* is not curve-derived right now" — because there is no curve, or because Curve's entry
conditions have not yet held for the hysteresis window — and the RPM PI runs in it regardless
(over the 15 s window it moves T\* by well under 0.1 °C at the gains below).

- **Curve.** Entered from `Held` when the fw-fanctrl view is fresh, the curve resolves a tread
  for `duty(fan_target)` (today's §2.6 derivation, including the ±snap rule and the duty↔RPM
  table), the replica is reconciled (`EC MISMATCH` clear), **and the debounced EC argmax is a
  controllable sensor** (`ARGMAX_DEBOUNCE_TICKS` as the prior §2.5/fwloop.10 did), all held for
  `ENTRY_HYSTERESIS_S` (15 s at the sample cadence). `T* = min(tread temperature, T*_ceiling)`
  with `T*_ceiling = min(cpu_hot_c − 2, gpu_hot_c − 2)`. Re-derived only when the **curve
  points or the snapped `target_duty` change** (the `points_changed` cache mode.rs keeps today —
  not on every `view_changed`, which fires ~1 sample in 30 in the field),
  shifting e_prev by ΔT* on both loops (no setpoint proportional kick,
  but measured-temperature changes still contribute). Pass `delta_tstar`
  for actual T* changes including Held PI updates. On an explicit upward
  fan-target/curve change in Curve, restore thermal to max without changing
  shadow or last_applied; the output rise slew governs recovery. Held's
  incremental updates do not perform this restoration.
- **Uncontrollable.** Entered from `Curve` or `Held` when the debounced argmax is an
  uncontrollable sensor (ambient, charger — plausible readings only, §2.1): the fan is being set
  by heat neither loop can touch (11–16 % of loaded field samples). T\* is frozen, both loops
  are ticked with `ThermalMode::Bypass` (§2.3: the thermal candidates report `max`, the shadows
  own the caps, so devices run at their draw plus headroom), and the `ArgmaxUncontrollable`
  flag is raised. Exit to `Held` (then `Curve` via the normal entry) when a controllable sensor
  takes the argmax (debounced). No cap step on either transition: §2.3's explicit mode-transfer seeds and
  one-tick motion suppression apply, including when thermal < shadow on entry.
  Bypass shadows ignore the frozen T* and follow load normally.
- **Stuck uncontrollable backstop.** While a known ambient/charger label continuously holds the
  debounced argmax, keep a rolling `ARGMAX_STUCK_DWELL_S = 300` s window of fresh 1 Hz
  observations. Once a complete window exists and its max-minus-min span is at most
  `ARGMAX_STUCK_SPAN_C = 0.25`, raise
  `ArgmaxStuck(label)` and quarantine that label from T* argmax selection and feasibility.
  This is a suspect-reading diagnostic, not proof of a failed sensor; stable real ambient
  heat can also trigger it, and the flag makes the conservative choice visible. Keep the
  reading in telemetry and the raw reconciliation stream. Leave `Uncontrollable` for `Held`
  through its normal cap-continuity transition, seed T* from the controllable-group max using
  the entry fallback/clamp above, and resync device errors. While any label is quarantined,
  keep `Held` with both loops in Regulate; inhibit both Curve and Uncontrollable entry so the
  same reading (or the other ambient/charger label) cannot immediately restore Bypass.
  Recover a quarantined label only after 30 consecutive fresh samples differ by more than
  0.5 °C from its value at quarantine; missing/implausible samples reset that recovery streak
  without clearing quarantine. Then resume normal entry debounce/hysteresis. Missing samples,
  resume and exit from argmax dominance reset a pending 300 s detection window; quarantine
  itself survives those events for the Auto session. Group temperatures and die/Tctl guards
  remain live throughout.
- **Held.** Entered from `Curve` when the view goes stale, the curve is invalid, or
  reconciliation fails (§2.6's `EC MISMATCH`), at Auto entry, or when a plausible argmax is
  Unknown or quarantined. Mid-session Curve loss retains the current in-memory T*; ordinary
  Uncontrollable exit retains its frozen T*, and the stuck backstop uses its explicit seed
  above. Only Auto/re-engagement reads the qualified persisted seed; a key change mid-session
  never reloads foreign persisted state. In Held, T* is driven by the
  **RPM PI**: `err_rpm = fan_target − fan_smoothed` (`fan_smoothed` = the existing
  `FAN_SMOOTH_N = 5` tail mean of `max(fan1, fan2)`, raw fallback on outage), a slow
  velocity-form PI in °C per RPM at `PI_PERIOD_S = 5`. **Tuning (roast d1):** the inner loops'
  closed-loop constant is λ_inner + θ ≈ 270 + 90 = 360 s; the cascade needs ≥ 4× separation, so
  λ_held = 1440 s → Kc = τ/(K·(λ_held + θ_eff)) = 35/(78 × 1530) ≈ 2.9e-4 °C/RPM at the
  reference slope, Ti = 35 s; output step bounded to 0.5 °C per PI tick. The plant gain
  K ≈ 78 RPM/°C holds at the curve's reference slope only, so the prior design's **slope
  schedule is retained for this loop**: Kc is scaled by `slope_ref / max(slope_at(T*),
  slope_ref)` clamped to `[0.25, 1]`, with the `0.25×` floor when no curve resolves a slope.
  The physical EC-autofan response is a separate plant property: its measured plateau has a
  near-zero slope in 67–73 °C and its rising branch is 140–420 RPM/°C below 64 °C. A resolved
  near-zero commanded-curve slope selects `1×`, not the `0.25×` floor. The
  effective closed-loop constant is therefore `λ_eff = λ_held / schedule` — up to ~96 min with
  the floor engaged — and §4 sim 6's bar is written against `λ_eff`, not λ_held (roast d2).
  Output clamped to `[T*_floor, T*_ceiling]`. For the nonempty set of plausible, known,
  non-quarantined uncontrollable readings, the raw floor is `max(readings) + 5 °C` and
  `T*_floor = min(raw_floor, T*_ceiling)`; a raw floor above the ceiling raises
  `TargetUnreachable(high)`. If that set is empty, define `T*_floor = T*_ceiling` and raise
  `EcUncontrollableUnavailable`: feasibility is unknown, so the interval conservatively
  collapses to the existing hot-guard ceiling instead of inventing an ambient reading.
  The same helper supplies entry-seed clamps and feasibility diagnostics on every tick.
  Restore the normal floor immediately when a usable uncontrollable reading returns.
  Directional conditional integration applies at both clamps, including the collapsed case. **Anti-windup — directional (roast d2):** the RPM PI's integrator is held **in the
  upward direction** when no device can raise its cap in response — every device's
  previous-tick hold is in `{Clamp(Max), Shadow, Bypass, ActuatorMismatch, GroupUnavailable}` (a shadow-bound device will not draw more because T\* rose; a frozen or
  unavailable group cannot move). `DrawUnavailable` is in neither directional
  blocking set: thermal regulation remains live, including after its dwell — and **in the downward direction** when no device can lower its
  cap — every device's hold is in `{Clamp(Floor), Bypass, ActuatorMismatch, GroupUnavailable}`.
  A `Shadow` hold blocks only the upward direction: lowering T\* pulls the thermal candidate
  below the shadow and does act. `Clamp(Floor)` blocks only the downward direction: raising T\*
  releases it. The sets are the complement of `None` split by direction; a new `Hold` variant
  must be assigned to one, both or neither explicitly. It reads the **previous tick's** Hold
  values (tstar.tick runs before the device ticks).
- **Released.** Fan invalid, EC invalid, watchdog trip: caps released to stock, both loops'
  integrators seeded on re-entry exactly as `handle_mode_transition` does today. Calibration is
  **not** a `TStarSource` state (roast d2): during a step test the controller does not tick the
  loops and holds both applied caps (§2.6); T\* and the state machine are frozen with them.
- **Feasibility / steepness (§2.7).** `TargetUnreachable (low)` when the tread duty is below
  the curve's minimum; `(high)` when T\* would exceed the ceiling (or the floor does, above).
  `SteepCurve` stays as an informational flag (the slope schedule above is the only consumer).
  A per-device `DeviceUnreachable` flag is raised when a device has sat at `Clamp(Max)` for
  `BOUND_HOLD_S = 60` with its group under T\* (informational — the load cannot reach the
  target) or at `Clamp(Floor)` with its group over T\* (the real one — the target is too cold
  for this load, or a sensor in its group is stuck high, §2.1).

`t_star_last_good` records the current valid controlled T* together with its current resolved
strategy, requested fan target and wall-clock save timestamp. It is written on Held exit, on
Auto exit, and at most every 60 s while dirty — never per PI tick. Do not refresh the saved
value or its age in Released, Uncontrollable, while both groups are unavailable, while a sensor
quarantine/unknown-label/empty-feasibility condition is active, or without a resolved strategy.
A mid-session strategy/target change rekeys future writes without reseeding the running loops;
the old record cannot be used under the new key.

**owns:** `TStarSource`, its states (incl. `Uncontrollable`), the RPM PI and its slope schedule,
the `ThermalMode` each loop is ticked with, `T*` persistence cadence. **consumes:** `FanctrlView`,
the duty↔RPM table, the replica's reconciliation verdict, `EcReading.argmax`/`all` for
controllability and feasibility, both loops' previous-tick `Hold`.

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
   ≤ 85 °C at half rate — **debounced over `CPU_HOT_STREAK = 3` consecutive samples** (roast d2:
   the fixed 53 W fast limit produces single-sample Tctl jumps of 10 °C+; `watchdog.rs`'s
   `TRIP_STREAK = 3` exists for the same reason). Both use the same `MaxRatchet` helper. The
   ratchet lowers `max` and the applied cap is always `min(cap, max)` — including a cap frozen
   by `ActuatorMismatch` (§2.3 step 6) — so a guard always has an effect on what is written. Each downward guard ratchet clamps thermal and its PI state to the new
   max, regardless of EC-group error, even during Bypass or Mismatch. This
   is guard evidence, never tracking to a draw-derived shadow. On recovery
   the ceiling alone rises; the PI state is not raised with it. Recovery
   gates use cpu_hot_c−5 and gpu_hot_c−4, respectively; test the branch
   where the die trips while the averaged group remains below T*;
4. `cpu.tick(...)`, `gpu.tick(...)` → two caps (with `ThermalMode` from the
   T\* source: `Bypass` in `Uncontrollable`, `Regulate` otherwise);
5. write through the existing actuator paths under the §2.3 write-cadence rule:
   `cpu.set_sustained_mw` with read-back (§2.9), `gpu.set_max_clock` with `verify_lock`; the
   read-back verdict is fed back into the next tick as `ActuatorState` (§2.3 step 6); the
   stickiness watchdog, the Mismatch re-write, the shutdown fences and the reassert paths are
   unchanged. **Verification is paired to sample time.** Record every successful GPU
   command with its monotonic completion time and generation. Before issuing
   this tick's write, score the clock sample against the latest completed
   command preceding that sample's acquisition timestamp. A sample with no
   matching command is Unverifiable. Allow the largest of that command and
   its immediate predecessor plus VERIFY_CLOCK_SLACK_MHZ (30) for the first
   one-second sampling interval after a downward change; afterward compare
   only the paired command plus slack. Do not reset strike streaks on a
   lock change; a compliant one-command-lag card must never strike during
   105 MHz/s descent. A persistently ignoring card still strikes after the
   bounded allowance. Resume clears pairing history and starts with an
   Unverifiable sample until the reassert has completed. CPU read-back
   remains paired with its synchronous write;
6. warm start: when both groups have sat within 1 °C of T\* and the fan within the steady
   window's tolerance for `STEADY_WINDOW_N` samples, record `(cpu_cap, gpu_lock)` under
   `WarmStart::key(strategy, duty, on_ac)` and refine the duty↔RPM table as today.

**Auto no longer requires calibration** (the `NOT CALIBRATED` gate on `SetAuto(true)` in
`controller.rs` is removed by the controller bead; the flag stays informational, raised when
either device has no fitted gains).

**Auto entry is bumpless by construction:** `draw` is taken as the mean of the last
`SEED_WINDOW_N = 5` valid samples (never one sample; the shadow's seeding is deferred while
`draw` is `None`, the loop running thermal-only until then). Each loop seeds its shadow at
`draw + headroom` and its thermal candidate at **`max`** (roast d2 — not at draw: a cool device
must not be capped at its running draw, which was the 2026-09-11 field complaint). The first
applied cap is therefore the shadow, `draw + headroom`, one headroom above the running state;
a device already above T* at entry performs the single entry handover
described in §2.3. A warm-start record is advisory: seed thermal at
max(recorded, clamp(draw + headroom, floor, max)) when draw exists,
otherwise defer its use. Thus a stale low record cannot impose a
minutes-long PI recovery or violate the entry headroom requirement.

Manual mode (`c`/`g` keys), Monitor, calibration, `p` release, quit and the emergency release
are unchanged in behaviour; they now address the two loops instead of the budget.

### 2.6 Calibration (`calib/`, change to §3.3)

`lut_sweep.rs` is deleted. `step.rs` becomes a per-device step test, run twice by the runner:

1. **Settle** (shared): both devices held at their current applied caps — "the settled cap" below — for the
   calibration's duration: the controller does not tick either `DeviceLoop` (no PI, no shadow,
   no handover; the T\* source is frozen with them) and writes nothing but the step and the
   restore; this is a controller-level freeze, not a `TStarSource` state (§2.4); the argmax controllable; both
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
(defaults are provisional and must pass the offline robustness gates;
the independent guards remain active before fitted gains are available).

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
`t_star_last_good: Option<TStarSeed { strategy: String, fan_target_rpm: u32,
value_c: f64, saved_at_unix_s: u64 }>` (for Held). `validated()` requires finite positive
in-range temperatures, a nonempty strategy and a sanitised-range fan target. At use time,
reject key mismatch, future timestamps and ages above 6 h (§2.4), then clamp the value to
current bounds. A legacy bare `t_star_last_good` number is dropped with one migration log
line; absent fields remain valid. The qualified record round-trips with its original timestamp,
so loading/saving unrelated fields does not renew its freshness. **Migration:** an old file loads with `lut` ignored (one log
line), `loop_gains` ignored (it was Mode A/B gains for one plant — not reusable), and any
`warm_start` value that is a bare number dropped (one log line). No `.bak` is written (carried
nit; out of scope). `Config` gains the shadow-cap keys (`shadow_headroom_*`, `shadow_fall_rate_*`, with the
positive sanitiser floors of §2.3; no band key), `gpu_shadow_enabled`, `cpu_hot_c` (default 90,
sanitised independently to `[82, CPU_TRIP_C − 1]` = `[82, 94]`: strictly below the CPU trip
and preserving `cpu_hot_c − 2 >= 80 °C`. GPU bounds remain independently sanitised by their
existing rule, so either device can determine `T*_ceiling = min(cpu_hot_c − 2, gpu_hot_c − 2)`.
The CPU exit/recovery thresholds remain `cpu_hot_c − 3` / `cpu_hot_c − 5`), and per-device gain overrides
(`cpu_gains` / `gpu_gains`, which take precedence over fitted gains — §2.3 Gains). `CPU_MAX_W` / `CPU_MAX_W_FLOOR` move from
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

`src/control/budget.rs`, `src/control/allocator.rs` (including the pinned tests
`CPU_PINNED_MARGIN_W` / `GPU_PINNED_MARGIN_MHZ` / `GPU_PINNED_UTIL_PCT` — rev3 has no pinned
test; `CPU_MAX_W`/`CPU_MAX_W_FLOOR` are rehomed first), `src/control/spike_antiwindup.rs`, `src/control/mode.rs` (replaced by
`tstar.rs`; the §2.6/§2.7 logic moves, not the file), `src/control/lut.rs`,
`src/calib/lut_sweep.rs`, the watts inner loop in `src/control/gpu_pid.rs` (the rate limiter and
directional conditional integration move into `DeviceLoop`), `DEMAND_MARGIN_W_*`,
`Freeze` (its `ActuatorMismatch` semantics move into `DeviceLoop` step 6), `Budget*` telemetry
fields, `Config::gpu_max_w`, `gpu_share_override`, `PersistedState::lut` and `loop_gains`
(`CPU_MAX_W`/`CPU_MAX_W_FLOOR` are rehomed in `config.rs` first), `mode.rs`'s
`ARGMAX_DEBOUNCE_TICKS`/`ENTRY_HYSTERESIS_S`/`points_changed` logic (moves into `tstar.rs`), the `gpu_watts_lut` test fixtures and the `ClockWattsLut` sims, the design
doc's §2.4/§2.5 (marked superseded with a pointer here), and every README/`docs/` mention of
budget, split, LUT, Mode A/Mode B.

## 4. Testing

- **Unit.** DeviceLoop: first-order plant response within 1% with ≤5%
  overshoot at defaults; directional unwind at each clamp; handover happens
  once on a measured hot crossing with previous Shadow selection, not on
  subsequent hot ticks, draw dips, or T* changes. Both shadow directions
  hold while hot in Regulate and run in Bypass even above frozen T*.
  A 60 s hot draw dip does not alter thermal except by its own PI;
  repeat crossings cannot rearm before 5 s nonnegative error.
  Test Floor/Max tie precedence and interior Thermal ties. Bypass entry
  from thermal < shadow and exit after shadow motion have zero transition
  step. Explicit upward Curve target change recovers through output slew;
  Held PI updates preserve thermal state and shift only the setpoint part
  of e_prev; measured-temperature P response survives coincident 5 s ticks.
  DrawUnavailable keeps thermal regulation live, becomes thermal-only after
  60 s without an output jump, and reappears without a step. GroupUnavailable
  absent/lost legs and guard precedence over Mismatch remain as specified.
  A resumed sample at t=100→7300, err=+20 changes no cap or PI output;
  subsequent ticks use bounded dt and the 5 s PI accumulator, not 5 samples.
  TStarSource tests every transition, hysteresis, directional Held
  anti-windup (DrawUnavailable + Shadow must not block upward integration),
  feasibility bounds, gain-source precedence and last-good persistence.
  Sensor and boxcar tests cover each group, per-group seeding, interval
  changes without reseeding, None and mismatch recovery. No automated test
  acquires physical hardware.
  Qualified last-good seeds: fresh/matching accepted; wrong strategy/target,
  expired, future-dated and legacy scalar rejected; unrelated saves never
  refresh age. Test empty/quarantined feasibility, sentinel-pair recovery,
  cpu_hot_c=82 yielding ceiling80 and default GPU ceiling86. Test rolling
  stuck detection, recovery, Held inhibition and Unknown argmax routing.
  Exact/prefix sensor groups, missing-label diagnostics, and unknown
  non-device labels are separate cases. Seed CPU50/GPU70/socket MA80
  at every reset: output CPU50/GPU70. Poll and strategy edits preserve
  history; shrink/grow retains the specified samples; 150 enters only raw
  reconciliation. Use synthetic hwmon fixtures; physical capture stays parked.
- **Simulation** (`ChainedPlant` grows a second thermal node: CPU heat → CPU group with
  τ≈35 s at 0.8 °C/W, GPU clock → GPU group with nominal τ=15 s and the
  `gpu_vr` tail represented by θ_eff and the τ=8–50 s robustness range,
  at **0.4 °C/W** (§2.3),
  cross-coupling 0.1 °C/°C each way; a scriptable GPU die temperature (the NVML reading the
  GPU HOT guard keys on) and CPU Tctl so guard episodes can be driven; fault injection for dGPU
  off, a single group's labels dropping out, a stuck-high sensor, fan outage, EC invalid / stale
  view).
  **GPU clock→draw model (no LUT):** `draw_w = load_level × P_full(requested_lock)`, where
  `P_full(requested_lock)` is the September full-load sweep as a piecewise-linear table —
  (1197 MHz, 49.3 W), (1402, 53.5), (1612, 64.2), (1807, 75.9), (1995, 90.8), (2143, 99.4),
  extended flat to 3090 at 100 W (the card's power limit) and linearly to (1000, 45) below —
  and `load_level ∈ [0, 1]` is the scripted GPU load; the reported SM clock is
  `min(lock, clock_at_power_limit(load_level))` — above the knee a loaded card runs BELOW the
  lock, as on hardware (lock 3090, card at ~2520 MHz), which is why the shadow (tracking the
  reported clock) bounds a power-limited card's cap near the knee (§2.3, "the GPU dead zone");
  GPU heat = `draw_w × 0.4 °C/W` into the GPU node (roast d2: the heat equation, not just the
  description, uses the GPU path's resistance). CPU: `draw_w = min(cap, cpu_load_w)` with
  heat `× 0.8 °C/W` into the CPU node, as today. The robustness sweep treats K as a
  **local incremental FOPDT gain about a declared physical pivot**, rather than changing the
  zero-power equilibrium: for the GPU, with `P0 = P_full(2143 MHz) = 99.4 W`, the robustness-only
  heat term is `0.4·P0 + 20·K·(draw_w-P0)`. The nominal `K=0.02` reduces exactly to the physical
  `0.4·draw_w` equation. The harness applies this affine transform only inside the thermal node;
  the controller and trace still receive the physical
  `draw_w = load_level × P_full(requested_lock)` and the separately power-limited reported clock.
  Target RPM, ambient, curve, load fractions, reported draw and clock, floors and ceilings are
  one immutable external tuple across the GPU K/τ/θ grid. Unit bars prove equal heat at P0 for
  every K, incremental slope `ΔT/ΔP = 20·K` above and below P0, and unchanged physical reported
  draw. CPU robustness uses one corresponding immutable external tuple across all ±50% cells.
  1. CPU-heavy, light GPU: CPU group settles at T\*, GPU sits at its shadow cap above draw,
     fans ±150 RPM of target ≥ 90 % of a 30-min converged window.
  2. GPU-heavy, light CPU: the mirror.
  3. Both heavy: both groups at T\*, same fan bar.
  4. Load step light → heavy on the GPU from a cold start (thermal candidate at `max`): cap
     ramps at the cadence-aware rising slew (an applied 1000 MHz in ≤ 8 s, with every request
     no higher than reported clock + headroom), no fan crest above target+250, **the GPU
     group's temperature overshoots T* by ≤ 4 °C, the GPU HOT guard does not trip, and the
     group settles within ±1 °C within 3 λ after the first downward knee crossing plus θ_eff**.
     Separately assert and report the crossing bound in §2.3 using the measured
     minimum hot error over the plateau and D at handover; never count this
     crossing as part of the 3 λ budget, the CPU cap unchanged during the step; and the same powered
     light → heavy edge repeated from a warm controller/thermal state — first converge at T\* under
     a feasible powered preload, then hold a powered light phase for a declared
     `0 < light_dwell < θ_eff` before the heavy edge. "Warm" qualifies the initial controller and
     thermal state; it does not claim the low-demand phase is itself a steady feasible T\* operating
     point. Require nonzero light draw and a measured heavy draw greater than 2× the light draw,
     GPU-group presence throughout, ±1 °C residency with Thermal selected during the scored
     pre-light warm window, no adjacent cap jump beyond the applicable slew, continuous Auto/Curve
     state, and no thermal-candidate reset through the light dwell; score
     warm overshoot from the heavy edge and require ≤ 2 °C. Repeat with gpu_shadow_enabled=false and with draw
     missing for >60 s before the step; record the full 947 MHz bound,
     require the same temperature/guard bars and post-crossing settling,
     and assert no output jump at disable/dwell/return. These are offline
     acceptance requirements, not measured passes.
  5. Auto entry under a steady heavy load: neither device's draw changes by more than 1.5 W /
     30 MHz in the first 10 s and, when the corresponding measurement exists, neither cap starts
     below `min(draw + headroom, active guard/config ceiling)`; fans do not fall. The absent-draw
     variant instead seeds from the applied cap without a step and remains thermal-only until the
     measurement returns.
  6. Curve loss mid-session from a converged state on the measured EC-autofan plateau uses the
     resolved near-zero commanded-curve slope's `1×` schedule: `Held` enters, T\* moves from the
     last good value, and fans return within ±150 RPM within 3 λ_held = 72 min. A distinct
     unresolvable-curve or genuinely steep (`slope >= 4 × slope_ref`) EC-autofan leg engages the
     `0.25×` floor and must return within 3 λ_eff = 288 min (λ_eff = 96 min), simulated offline at
     plant speed. Run the same period-agnostic hunting check over both clocks; curve return → `Curve` with
     no step in either cap. Restart variants use a matching fresh seed, a different strategy,
     a changed requested fan target, an expired (>6 h) timestamp and a future timestamp with
     no curve available: only the matching fresh record is accepted, every other variant
     starts from the current controllable-group max/clamp, and the per-group first averages
     equal their own instantaneous maxima rather than the socket argmax MA.
  7. Unreachable device: compare against an identical thermal/load replay without the GPU fault;
     a GPU that cannot reach T\* leaves every CPU decision and cap equal to that reference while
     raising `DeviceUnreachable` only for the GPU.
  8. Configuration smoke: each `TStarSource` state (incl. `Uncontrollable`, via an
     ambient-dominated argmax leg) and each `Hold` / `Selected` value is reached at least once
     across the sims (the checklist is behavioural, not self-pushed vectors). Observe all
     `TStarState` values, including `Released`, from actual `TStarSource` outputs; observe
     `TStarFlag` diagnostics there as well. The controller intentionally publishes
     `tstar_state = None` outside Auto and therefore never emits telemetry state `Released`.
     Observe every `Hold`, `Selected` and schema-v3 `TelemetryFlag` key/polarity exclusively from
     actual controller telemetry. Cover semantic variant keys exhaustively:
     label strings are payloads, finite device/bound combinations are separate where behavior
     differs, and `Legacy` is one semantic key with a representative payload. Exercise active and
     clearing polarity for dynamic structured flags where the emission boundary supports both;
     legacy controller `StatusFlag` is covered by its own exhaustive tests. Expected-key mappings
     use no-wildcard exhaustive matches, while observed keys come only from the named source or
     controller boundary.
  9. Hot-guard episodes: (a) GPU — a 5-min die-temperature excursion above 88 °C: the ratchet
     reaches the floor, recovery of `max` starts only below 84 °C, no re-trip within the
     episode's tail, post-episode fan overshoot ≤ 150 RPM (the prior design's bar) and the
     applied cap back within 10 % of its pre-episode value within 3 λ after
     recovery eligibility: the die remains below its recovery threshold,
     load and T* return to the prior feasible operating condition, and max
     has reopened to the pre-episode cap. Report guard-clear, recovery-gate
     and ceiling-reopened times separately
     (thermal was clamped by the guard independently of group error).
     Run both group-already-hot and group-still-cool branches; use a compliant
     one-command-lag verifier in both and assert zero mismatch strikes or
     mismatch-driven releases. A separate ignoring-card leg must still trip; (b) CPU — the mirror on Tctl with
     `cpu_hot_c` 90: a 3-sample streak trips, a 1-sample spike does not, recovery uses the same eligibility and 3λ bar as the GPU. With a
     positive error held constant (or nondecreasing at PI sample instants) at e_min, separately bound the first full 0.5 W
     rise by 0.5/(Kc/Ti*e_min) plus one PI period and write interval; do not
     promise a 60 s floor exit for an arbitrarily small positive error.
  10. Robustness: sims 1–3 repeated with the CPU plant's K, τ and θ each
      perturbed by ±50 % and with the GPU's full crossed grid from §2.3
      (τ=8,15,25,50; K=0.01,0.02,0.03; θ_eff=45,90,135), plus a
      **fluctuating-load leg** (roast d2): sim 3's load with square-wave half-cycle dwell
      durations of 60 s and 300 s (full periods 120 s and 600 s) at fixed T\*. Choose amplitude
      so the live controller-group telemetry, after its 60 s boxcar, reaches both +4 °C and
      −4 °C about the live T\*; raw plant temperature does not satisfy this premise. Also run a T* down-and-back step
      (fan-target change in Curve at unchanged load), plus a 60 s load dip
      beginning while each group is still above T*. For the target-return
      leg, the cap returns within 10% of its prior steady value within 3λ;
      for the hot-dip leg compare against an identical thermal-error replay
      without the draw dip: PI state must match (no draw-driven loss), and
      any purely shadow-limited applied 1000 MHz recovery takes ≤8 s once err≥0,
      while each request remains at most the reported clock plus headroom.
      Include the corresponding Held setpoint excursion with a 3λ_inner
      recovery window measured after T* returns to its original value.
      Apply a period-agnostic **no-relay rule** on every 30-min
      window of every leg (no sustained oscillation of the cap or the fan with a peak-to-peak
      above 100 RPM / 4 W / 150 MHz at any period from 30 s to 20 min beyond what the load's
      own period forces) — a loop hunting slowly inside ±150 RPM does not pass. For the
      square-wave legs, residualize against a separate forcing-only reference trace keyed by
      forcing phase; do not fit the tested trace to itself and do not exempt the forcing period
      or any neighboring period. Synthetic grader bars add an independent relay at the same
      period and at a nearby period to the known forcing response and require both to be found.
  11. Stuck-high sensor: for the startup-after-seed leg, one valid Auto-entry sample first
      establishes the actual applied seed, then the GPU-group label is pinned at 105 °C on the
      immediately adjacent next 1 s sample with no intervening PI tick or actuator write;
      separately pin it from t = 5 min. The GPU cap goes to its floor (the safe direction, §2.1),
      `DeviceUnreachable` (real) is raised within `BOUND_HOLD_S` after reaching
      the floor; bound travel to the floor from the actual seeded output by
      D/(Kc/Ti*e_min) plus the PI/write latency while error remains ≤−e_min.
      The other device-group input is unaffected. Unstick the GPU label to
      the live plant reading, restore the prior feasible load and target,
      and require cap/temperature recovery within 3λ after the group boxcar
      has flushed the bad readings. Repeat with a 150 °C label: `EcImplausible`
      names it, plausible control maxima/argmax omit it, while the emulator and replica both
      retain it in the positive-only raw reconciliation stream. Over at least four scored
      steady views, no `EC MISMATCH` is caused by the gate and the valid control argmax does
      not force Held or Bypass. Assert no direct device-error/cap change from the excluded
      value; any physical fan response commanded by fw-fanctrl remains visible and is not
      falsely claimed absent. Add ambient and charger variants pinned at 105 °C from startup
      and from t=5 min: within the debounce plus 300 s unchanged dwell, `ArgmaxStuck(label)`
      appears, Held/Regulate replaces Bypass, the suspect reading is excluded from feasibility,
      and further unchanged samples cannot re-enter Uncontrollable. Unstick it by >0.5 °C for
      30 consecutive samples and verify quarantine clears, normal entry timers resume, and
      independent guards remain effective. A separate pair of −150 ambient/charger sentinels
      with valid CPU/GPU readings keeps EC valid, raises `EcUncontrollableUnavailable`, and
      produces a finite collapsed Held interval until a plausible label returns.
- **Hardware (the user's, parked):** the 30-min gaming check; the VR/VRAM label spike.

## 5. Follow-ons (not in this tree)

- `state.json.bak` on migration; the telemetry `loop_mode` column and `Decision.reasons`
  consumer from the earlier roast nit list; the Instant-vs-suspend clock escalation (design doc
  §270 vs `sampler.rs`), still open.
- The upstream fw-fanctrl `movingAverageInterval` change (60 → 20 s) as a way to shorten the
  dead time — an operator config decision, not code.

## Post-Implementation Notes

*As this design is implemented and iterated on — bug fixes, adjustments, anything that diverged from the assumptions above — append a dated note here, whether or not a formal debugging skill was used.*

### 2026-09-12 — final integration sweep

- The end-to-end fake/plant flow now proves that calibration saves valid CPU and GPU fits under the live `quiet16:60` key, a qualified T* retains its original timestamp across reload, and 45 qualified live controller samples produce a non-bound paired warm start. A restarted controller consumes that exact saved pair and both fitted gain records; an otherwise identical default-gain controller produces a different PI response, proving the fitted gains are installed rather than merely labelled. A separate 7,200 s resume flow proves that both applied caps are held and immediately reasserted without creating an actuator-mismatch hold.
- The sweep found and fixed two small controller seams. The controller had bounded elapsed time to seven seconds before calling `TStarSource`, which made the source's unmarked-wall-gap reset unreachable; it now passes the raw finite positive delta while `DeviceLoop` keeps its own two-second bound. `TStarSource` also freezes and resets its gate/debounce dwell and every quarantine-recovery streak before a resumed or unmarked gap sample, so missing wall time cannot enter Curve or complete a 30-sample recovery. `NotCalibrated`, whose meaning is scoped to the active Auto key, is now cleared with the other Auto-owned flags on exit instead of leaking into Monitor.
- The Sim 8 enum-derived smoke now binds each important observation to its scripted interval: unknown argmax selects Held/Regulate, known ambient/charger argmax selects Uncontrollable with both loops in Bypass, and GPU group loss progresses through GroupUnavailable and GroupLost before clearing into finite regulation on return. The pre-existing Sim 8 registries exercise every `TStarState`, `Selected`, `Hold`, T* diagnostic and structured telemetry flag; TUI fixture tests render every state, binding, hold, gains source, legacy status flag and structured flag.
- The `Config` integration destructure is exhaustive and a mechanical registry checks every top-level and LED key against a non-comment runtime consumer in controller, main or LED code. The exhaustive `StatusFlag` registry checks each variant's production raise expression, passes it through the same Decision flag mapper used by `apply_effects`, serializes it, and invokes the real TUI label renderer. The production `apply_effects` test emits a live Auto v3 decision with T*, state, both complete device records, applied caps and flags, and asserts every retired decision key is absent.
- Elapsed-time, setpoint-delta and resynchronisation paths are covered at `TStarSource`, `DeviceLoop` and Controller boundaries, including the new unmarked-gap regression and the resumed plant flow. `requested_slew_accumulates_each_sample_without_an_applied_note` separates per-sample requested slew from applied feedback; the controller cadence tests cover successful and failed CPU/GPU writes and the two-second completion-time boundary.
- The complete offline acceptance suite was reused rather than copied: nominal CPU/GPU/both runs, the CPU and full 108-cell GPU tau/K/theta robustness matrices, Sim 4 knee timing, Sim 9 guard eligibility/recovery, and Sim 11 raw reconciliation/quarantine/recovery all ran in the full gate. The final elevated `cargo test --no-fail-fast -- --format=terse` result was **695 passed, 0 failed, 2 ignored** in 162.77 s. Focused results were integration tests **9 passed**, controller tests **119 passed**, T* source tests **41 passed**, retained controller simulations **3 passed**, and the telemetry-field filter **1 passed**. `cargo clippy --all-targets -- -D warnings` passed, as did `git diff --check`.
- The deletion audit found no retired module basename. Current-source hits for retired decision keys occur only in negative serialization assertions; `gpu_max_w` occurs only in its explicit config migration warning/test. No open offline item or blocker remains. No hardware was accessed: the 30-minute gaming check and VR/VRAM label-spike check remain parked for the user, and the two corresponding tests remain ignored.


### 2026-09-12: measured GPU defaults and calibration outcomes

User-approved field revision: remove cross-device temperature rejection entirely. With the fan curve live, increasing CPU power can cool the GPU without a workload change. Settling still checks both groups; response fitting requires only the primary group trace. Verified caps, response coverage, thermal guards, minimum response/time constant, and the 0.25×–4× gain band remain.

Use run-1789252085’s GPU model as the runtime default: K=0.009527650224779704 C/MHz, tau=32.564086253945035 s, fitted theta=38.45154292954405 s at MA=60 s. Subtract the 30 s MA contribution before adding the live interval’s contribution, yielding Kc=22.22180481777582 at MA=60. CPU defaults remain unchanged. The user explicitly selected this despite an 11.12 C overshoot on the previous provisional K=.02/tau=15/delay=90 model; actual control validation on the machine remains pending. Nominal response tests now use the measured model. The historical robustness matrix retains its explicit GPU gains (2.1, 15) and must not be cited as validation of this default.

The TUI suggests gpu_burn while maintaining >90% GPU load, then retains success/partial-success/failure details until dismissal or the next calibration. Results include old/new calibration gains, specific rejection values and limits, abort reasons, persistence errors, and any config override preventing fitted gains from controlling the device. Recorded primary response traces are preserved as a regression fixture under src/calib/fixtures/2026-09-12-responses.csv.


### 2026-09-12: shared read-first CPU cap maintenance

Field run-1789256681 showed CPU draw jumping15W to52W and settling40W while calibration reused a verification flag from268s earlier. User approved shared read-first maintenance for normal Auto and calibration. CpuActuator now reads slow/fast limits first, repairs only a confirmed mismatch with a post-read shutdown/thermal fence, and preserves both the original verdict and repair result. Normal operation uses the existing10s schedule and feeds maintenance verdicts through Auto’s mismatch/blind/release handling. GPU verification/reassertion remains as before.

Calibration refreshes CPU evidence every10s and before concluding a response; schema5 records the read-back timestamp/result and reset reason separately from command completion. A detected reset discards the affected response, restores the baseline pair, restarts its settling window, and retries at most twice per device. Retry notes remain visible without mislabelling eventual success as rejection. An unreadable CPU check during a response fails explicitly. The cause of the platform’s reset remains unknown; maintenance recovers from it without claiming an external culprit.

### 2026-09-12: compact control header

User-approved display simplification moves the target/state and per-device signed errors/gain sources into the existing cap header and removes the separate eight-row control panel. Watts use one decimal and GPU caps/floors use one-decimal GHz. Cyan up / amber down indicate desired temperature direction (target minus group), not measured trend; displayed zero is green. Raw thermal/shadow candidates and hold details remain in telemetry. Warnings precede optional details on narrow terminals.

### 2026-09-12: board heat and current-target hot handoff

The recorded Auto run showed a sustained ~4,000 RPM plateau for a 3,500 RPM request because ambient + 5°C forced T* above the curve-derived target. Remove that lower bound, retaining a numeric 0°C bound and the guard-derived upper ceiling. Missing board readings no longer collapse the temperature range to the ceiling. Ambient/charger dominance now selects Held RPM feedback with Regulate, never label-triggered bypass. The existing sensor diagnostics and quarantine remain.

Declare a settled fan limitation only with both device floors and 60 seconds of CPU/GPU temperature spans within 1°C/2°C and fan span within 150 RPM, with fans above target + 150 RPM. Reset evidence across missing data, resume/gaps, cap release, or fan-target changes. Hot handoff uses current negative error and prior Shadow selection, once per armed episode with five cool seconds to rearm; draw-recovery reseeding retains precedence. Calibrated gains and thermal guards are unchanged.

The UI bundles the compact header with ambient/NVMe readings. Simulation curve-loss fixtures now warm up for 7,200 seconds before the disturbance (previously 3,600), retaining all per-sample convergence tolerances and post-disturbance deadlines.

### 2026-09-12: hot headroom trim and 30-second averaging

User requested both changes after the improved controller took roughly ten minutes to converge. At an armed hot handoff with five seconds of continuous valid draw history, trim thermal output to at most the recent peak plus 2 W / 100 MHz (bounded by the configured shadow margin), never above the applied cap. A short draw dip cannot seed an excessively low limit. Preserve hardware floors and slew from the applied cap rather than a pending request; retain ordinary PI operation thereafter. Missing data, resume, timing gaps, and actuator mismatch clear the history. Initial Auto entry and ordinary shadow margins are unchanged. The local quiet16 movingAverageInterval was changed from 60 to 30 via fw-fanctrl set_config and verified in memory and on disk. Calibration keys remain interval-specific; no 60-second fit is copied to 30 seconds.

### 2026-09-12: target history on every chart

User requested time-varying target overlays instead of horizontal lines representing only the latest target. The model snapshots the latest controller-reported fan target, T*, CPU cap, and GPU cap on each measurement event into matching rolling rings. Status updates do not mutate existing history. All four charts render those series, splitting unavailable or released temperature/cap targets into gaps and including target history in temperature/fan axis bounds.

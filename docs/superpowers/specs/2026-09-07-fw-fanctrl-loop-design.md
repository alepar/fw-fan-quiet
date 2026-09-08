# Closing the loop on fw-fanctrl (no learned thermal model)

**Date:** 2026-09-07
**Status:** draft (root spec of a `super-design` run, no-beads mode)
**Input:** `docs/research/05-fw-fanctrl-loop.md` (verified research notes), prior designs in
`docs/plans/` (2026-07-03 base design, 2026-07-09 Kalman adaptation, 2026-07-10 raise gate,
2026-07-14 drain veto).

## Goal

Auto mode holds the fans at the user's RPM target by capping CPU and GPU power through one
integrator that regulates the temperature fw-fanctrl reacts to (Mode A), falling back to
regulating measured RPM (Mode B) whenever the temperature loop is unusable, with no learned
power→RPM thermal model in the binary. Observable outcome: on a 30 minute gaming session the
measured fan RPM stays within ±150 of the snapped target for ≥90 % of the converged window, auto
survives fw-fanctrl outages and strategy edits without a step in the commanded caps, and
`thermal_model.rs`, `kalman.rs`, `trust.rs`, `cooldown.rs`, the calibration matrix phase and the
controller's adaptation tier no longer exist.

## Decisions taken with the user (2026-09-07)

| Question | Decision |
|---|---|
| Loop mode | **Mode A primary** (PI on the EC max-temperature moving average, setpoint from the inverted live curve) |
| Fallback | **Mode B** (PI on measured RPM) in scope, bumpless switching |
| Target unit | **RPM**, snapped to the integer-duty grid via a duty→RPM table |
| Allocation | **Demand-driven** split of the scalar budget (existing `allocator::demand` scoring survives) |
| NVMe guard | **In scope**, same shape as the required dGPU guard |
| fw-fanctrl inactive / socket dead | **RPM fallback** while fans respond; **release to stock** if the RPM loop is invalid too |
| Actuator read-back | **In scope**: read back every write, mismatch = actuator failure |
| Tuning | **Fixed IMC defaults + a step-test calibration phase** that fits FOPDT and derives gains |
| EC temperature source | **cros_ec hwmon replica** of fw-fanctrl's max rule, reconciled against the socket periodically |
| Persisted state | **Integrator warm-start keyed by (strategy, target duty, AC)** plus LUT, gains, duty table |

## Facts verified on bazerame while designing

- The fw-fanctrl socket answers without root; `print all` returns `strategy`, `speed`,
  `temperature`, `movingAverageTemperature`, `effectiveTemperature`, `active` and the full config.
- `framework_tool --thermal` (v1.1.0 build here) prints `F75303_Local`, `F75303_CPU`,
  `F75303_DDR`, `APU`, three dGPU sensors at 0 and `dGPU temp: NotPowered`. **No battery line.**
  Values are printed as integers (rounded).
- cros_ec hwmon (`/sys/class/hwmon/hwmonN`, name `cros_ec`) exposes `ambient_f75303@4d`,
  `charger_f75303@4d`, `apu_f75303@4d`, `cpu@4c`, `gpu_amb/gpu_vr/gpu_vram_f75303@4d` (read −150
  when the dGPU is off) and `gpu_temp@40` (ENODATA when off). At one instant the hwmon max was
  47.85 °C and the socket reported `temperature: 48.0` — the replica rule is max over positive
  readings, **rounded** to integer °C.
- `nvme` hwmon exposes `Composite`; `k10temp` exposes `Tctl`; the battery has no temperature
  attribute in sysfs.
- Live curves (2026-09-07): `quiet16` = (0,15) (55,15) (65,21) (75,31) (82,37) (88,55) (95,100),
  `movingAverageInterval` 60; `cool16` = (0,20) (50,20) (60,30) (70,42) (85,100), interval 60.
  Verified truncation case on cool16: T_eff 51.8 → duty 21.
- **Measured 2026-09-08 with the dGPU powered** (NVML: 18.9 W, P0, 38 °C): the cros_ec `gpu_amb`,
  `gpu_vr` and `gpu_vram` sensors still read −150 and `gpu_temp@40` still returns ENODATA. They
  are not merely "off while unpowered" — they never report on this machine, confirming the
  research doc's original finding that the dGPU does not feed the EC fan curve. The replica's
  rule is nonetheless "every positive reading joins the max", which matches fw-fanctrl's own
  regex and costs nothing if a future firmware makes them live.
- **Measured 2026-09-08, dGPU thermals (NVML):** `GPU Target Temperature Specification` is
  **87 °C**, and the driver reports headroom against that same reference. The card deliberately
  runs to 87 °C under sustained load, so any guard threshold below it fires during normal
  gaming.
- **Measured 2026-09-08 under load:** cros_ec max 74.85 °C (argmax `cpu@4c`) against the socket's
  `temperature: 75.0` — the round-to-integer max rule holds under load as well as at idle.
- **Measured 2026-09-08, duty→RPM seed validated in place:** under `quiet16` at EC max 75 °C the
  curve gives duty 31 and the fans ran 2649 RPM (30 samples, spread 2642–2656). The seeded table
  interpolates 2638 RPM at duty 31, an error under 0.5 %. The seed is good enough to ship, which
  is what makes passive refinement a refinement rather than a dependency.
- **Measured 2026-09-08, does fan speed cool the SN850X?** Sustained O_DIRECT reads, aborted at
  an 82 °C safety cap after 50 s (`docs/research/2026-09-08-nvme-airflow-probe.csv`). At
  4748 RPM the drive still went 66.9 → 79.9 °C in 30 s and kept climbing; after pinning 20 %
  duty (1694 RPM) it reached 82.9 °C. The run never plateaued, so it is **not** a clean
  steady-state comparison of low versus high airflow. What it does establish: near-maximum
  airflow did not hold the drive, and the EC maximum fell from 74 to 69 °C throughout, i.e. the
  SoC cooled while the drive roasted. That is the "SSD hot, SoC idle" case, observed directly.
- **Measured 2026-09-08, the EC's own autofan curve** (fw-fanctrl paused, CPU load ramp, 300
  samples): a staircase that saturates early. Median RPM by EC max was 4096 at 61–62 °C, 4520 at
  63 °C, 4658 at 64 °C, then **flat at ~4748 RPM across 67–73 °C**. Two consequences, both
  load-bearing for Mode B under `active: false`: the EC is far more aggressive than `quiet16`
  (4748 RPM where fw-fanctrl asks 2649 at a *higher* temperature), and in the normal operating
  band its slope is ≈ 0, so **watts have almost no authority over RPM there**. Below ~64 °C the
  same curve is steep, 140–420 RPM/°C. Either regime is far from the plant Mode B's gain was
  identified against.
- `ryzenadj --info` fails while `ryzen_smu` is loaded — the same precondition the existing
  `smu_module.rs` already enforces for writes, so read-back shares it.
- `nvidia-smi` reports `power.limit` N/A and `enforced.power.limit` 100 W; we control locked
  clocks, not power, so the GPU read-back is the measured SM clock under load.

## 1. Architecture

Cascade, outermost first:

```
target RPM ──(DutyRpmTable, snap)──▶ target duty d
target duty d ──(inverted live curve)──▶ T* (tread centre) + slope
T* − MA(EC max)  [Mode A]  ─┐
target_rpm − rpm_smoothed [Mode B] ─┴──▶ PI (velocity form, no D, 5 s) ──▶ budget_w
budget_w ──(split_budget: floors first, then ∝ demand)──▶ (cpu_w, gpu_w)
cpu_w ──▶ ryzenadj --slow-limit (+ read-back)         gpu_w ──▶ GPU PI (1 Hz, unchanged) ──▶ NVML clock lock
```

`rpm_smoothed` is the controller's existing `FAN_SMOOTH_N = 5` tail-mean of `max(fan1, fan2)`
over the fan window (raw fallback on a short outage), and `fan_valid` is the sampler's existing
flag; both survive unchanged. The user's RPM target is snapped through the table and the curve's
tread fallback (§2.3) into `target_duty` every allocator tick. Nothing modifies that target: the
guards of §2.8 either act on the budget split (dGPU) or only report (NVMe).

fw-fanctrl remains the only process that commands the fans. The controller never writes to the
socket; it only reads (`print speed`, `print all`).

The combined loop has exactly one integrator (fw-fanctrl is P-only), which is what makes the two
controllers on one measurement benign. Never add a second integrator on the same sensor.

## 2. Components

### 2.1 `fanctrl/` — socket client and curve model (new)

- `FanctrlClient`: AF_UNIX stream to `/run/fw-fanctrl/.fw-fanctrl.commands.sock`, one command per
  connection, send the raw CLI string, read to EOF, parse JSON. Connect timeout 1 s, read timeout
  3 s. Socket path is a config key with that default.
- Polling cadence: `--output-format JSON print speed` every allocator tick (5 s; no fork on the
  daemon side); `--output-format JSON print all` every 30 s (two forks on the daemon side — never
  faster). All socket I/O happens off the control thread; results are delivered as a `Sample`
  field group (`FanctrlView`) with a monotonic timestamp.
- Commands are a closed enum `PrintCommand { Speed, All }` — the client cannot send anything
  else, and the fake records every command so tests can assert the socket stays read-only.
- `FanctrlView { strategy, active, speed_pct, temperature, ma_temperature, ma_interval, curve:
  Vec<(temp, speed)>, observed_at, all_observed_at }` — `curve` is the raw point list of the
  resolved strategy (the client owns `resolve_curve(print_all_json, strategy) -> Vec<(f64,u8)>`,
  matching the `strategies` map by exact name); consumers build a `Curve` from it with
  `Curve::from_points`. `observed_at` is stamped by any successful poll, `all_observed_at` only by
  a successful `print all`; reconciliation keys on the latter. Staleness: no successful
  `print all` for 90 s or no successful `print speed` for 15 s ⇒ `stale`. Connection refused /
  ENOENT ⇒ `absent`.
- `Curve`: piecewise-linear in **file order** (never sorted), flat clamp below the first and above
  the last point, `int()` truncation — a faithful copy of `FanController.py`. Provides:
  Non-monotone curves are rejected: `from_points` returns an error on any descending segment,
  because a duty's preimage would then be a union of intervals, `tread` would be ill-posed, T*
  could land on a falling segment, and the steepness test would never fire on a negative slope.
  A rejected curve means TempLoop is unavailable and the arbiter falls to RpmLoop with
  `STEEP CURVE` raised for visibility.
  - `duty_at(t: f64) -> u8`
  - `tread(d: u8) -> Option<(t_lo, t_hi)>`: the maximal temperature interval where
    `duty_at(t) == d`; `None` when no temperature yields exactly `d` (the target then snaps to
    the nearest reachable duty, see 2.3).
  - `t_star(d) = (t_lo + t_hi) / 2`.
  - `slope_at(t) -> f64` in %/°C (the segment's slope; 0 on the flat clamps).

### 2.2 `sensors/ec.rs` — fw-fanctrl sensor replica (new)

- Discovers the `cros_ec` hwmon chip by name (same discovery as `hwmon.rs`), reads every
  `temp*_input` with its `temp*_label` at 1 Hz.
- `EcReading { max_c: i32, argmax: EcLabel, all: Vec<(EcLabel, f64)> }`: drop readings ≤ 0 or
  ENODATA, round each to integer °C, take the max. Ties resolve to the first label in sysfs order.
- `EcLabel` classifies by label prefix: **controllable** = `apu`, `cpu`, `gpu_*` (if one ever
  reports, the GPU cap moves it); **uncontrollable** = `ambient`, `charger`. Every **positive**
  reading joins the max, mirroring fw-fanctrl's own rule; on this machine the `gpu_*` sensors
  never report (measured with the dGPU powered, §Facts), so in practice the max is over `apu`,
  `cpu`, `ambient` and `charger`. A `temp*_input` that fails to read or parse is dropped exactly
  like a ≤ 0 reading; the sampler tick is 1 Hz, which is what sizes the boxcar in samples.
- **Seeding and invalidation.** `EcAverage` is seeded from `view.ma_temperature` on every auto
  engagement, every re-engagement from `Released`, at calibration exit, and whenever
  `Sample.resumed` is set (the existing resume signal) — a monotonic clock does not advance
  across suspend, so pre-suspend samples and freshness stamps are stale on wake and are
  discarded along with the fan and steady windows. Until the first seed the arbiter's
  reconciliation state is **unreconciled**, which is not the same as mismatched: TempLoop is
  unavailable until the first successful comparison, and the boxcar must hold at least
  `ma_interval` samples (or a seed) before its mean is used as one.
- `EcAverage`: boxcar over the last `N` **non-zero** samples, `N` = `ma_interval` from the
  socket (capped 100, like fw-fanctrl's deque). Mirrors fw-fanctrl's off-by-one: the value used at
  tick n is the mean of samples n−N..n−1. The buffer is **not** cleared on strategy change (fw-fanctrl
  keeps its buffer too); it is cleared only when the replica is re-seeded after `EC MISMATCH`
  clears.
- `ec_valid: bool` on the `Sample`, false when the chip is missing or every reading is dropped.

### 2.3 `DutyRpmTable` (new, in `fanctrl/`)

- `BTreeMap<u8 duty, f64 rpm>`, seeded from the measured points (15→1195, 20→1670, 27→2300,
  30→2560, 36→3030, 40→3380, 44→3670, 48→3950, 52→4180, 85→5920); linear interpolation between
  entries, flat clamp outside.
- `duty_for_rpm(target) -> u8`: nearest tread by interpolated RPM; ties go down (quieter).
- `rpm_for_duty(d) -> f64`.
- Passive refinement: when the controller observes a steady window (population stdev of the
  **smoothed** RPM series < 60 over ≥ 40 s **and** `active: true` **and** the fw-fanctrl view's
  own `speed_pct` equals `target_duty` for the whole window **and** no guard override is active
  **and** `u` is off both bounds), the entry
  for that duty is updated `rpm ← 0.8·rpm + 0.2·mean` (created if absent). A refinement is
  rejected when `|mean − rpm| > 25 %` of the current value, and clamped so the table stays
  strictly increasing in duty. `DutyRpmTable::default()` is the ten seeded points and is the
  serde default, so a legacy `state.json` without the field loads the seed. Persisted in
  `state.json`. A refinement that changes `duty_for_rpm(target)` re-derives T* (bumpless: the
  budget is untouched; the integrator's previous error is re-synced, §2.4). The `speed_pct`
  condition matters because the controller commands watts, not duty: during a `GPU HOT` episode
  or at a budget bound the duty fw-fanctrl actually runs can sit a tread away from the one the
  target names, and writing that window's RPM into the target's entry corrupts the table by a
  full tread, which the 25 % rejection band is far too wide to catch.
- If `tread(d)` is `None` for the snapped duty (the curve skips that integer), snap to the nearest
  duty with a tread, preferring lower.

### 2.4 `control/budget.rs` — the single integrator (new)

- Velocity-form PI, period `PI_PERIOD_S = 5` (the allocator cadence), no derivative term.
  `Δu = Kc·(e_k − e_{k−1}) + (Kc·Ts/Ti)·e_k`; `u` is the total budget in watts.
- `u ∈ [cpu_floor_w + gpu_floor_w(lut), cpu_max_w + gpu_max_w]` where `gpu_floor_w` is the LUT's
  watts at `gpu_floor_mhz`. Clamping anti-windup plus back-calculation with `Tt = Ti`.
- Error source is a `LoopError` enum supplied by the arbiter each tick:
  `Temp { e_c: f64 }` (T* − MA, gains `kc_w_per_c`, `ti_s`), `Rpm { e_rpm: f64 }`
  (`rpm_for_duty(target_duty) − rpm_smoothed` — the **snapped effective** target, so Mode B
  regulates the same quantity Mode A does and cannot hunt between adjacent duties; gains
  `kc_w_per_rpm`, `ti_rpm_s`). **Both integrate into the same `u`**, so a mode switch changes
  only the next increment — bumpless by construction. `Budget::resync_error(e)` resets `e_{k−1}`
  to the current error without touching `u`; the controller calls it on a mode switch **and on
  every T* or snapped-target re-derivation** (strategy edit, curve edit, table refinement, NVMe
  boost) so the velocity form never converts a setpoint jump into a proportional kick.
- `Budget::new(&LoopGains)` / `set_gains(&LoopGains)`: the controller loads
  `PersistedState.loop_gains` on auto entry, `LoopGains::default()` when absent.
- `Freeze` reasons (no integration, `u` held): `ActuatorMismatch`, `Calibrating`, `Released`.
  Freeze is reported in the decision telemetry. (Socket staleness, `active: false` and an
  uncontrollable argmax are not freezes — the arbiter moves the loop to RpmLoop instead, §2.5.)
- `Gains`: persisted `LoopGains { kc_w_per_c, ti_s, kc_w_per_rpm, ti_rpm_s, tau_s, theta_s,
  k_c_per_w, k_rpm_per_w, fitted_at }` or the defaults below. The regulated variable is the
  boxcar mean, not the raw EC reading, so the effective dead time is `θ + N/2 ≈ 50 s`, not 20 s;
  the defaults are derived with that `θ_eff` (λ = max(90, 3·θ_eff) = 150), giving
  `kc_w_per_c = 0.22`, `ti_s = 35`. The λ = 90 floor is **not** the boxcar's margin — it is a
  separate lower bound — and the same `θ_eff` substitution applies to the fitted gains in §3.3.
- **Mode B's gain is scheduled on the live curve, not fixed.** `kc_w_per_rpm` assumes a
  RPM-per-watt plant gain that scales with the curve's slope at T*: a flat quiet16 tread and a
  steep cool16 tread differ by roughly a factor of four. The controller therefore scales the
  persisted or default `kc_w_per_rpm` by `slope_ref / max(slope_at(T*), slope_ref)` with
  `slope_ref = 1.0 %/°C`, clamped to [0.25, 1]×, so the steepest supported tread gets the
  smallest gain. This is the minimal form of the gain scheduling §6 defers; without it the
  cool16 × RpmLoop acceptance run is required to pass on a plant four times more sensitive than
  the one the default was derived for.
- Warm-start: **only on auto entry** (and on re-engaging from `Released` or at calibration
  exit) the integrator seeds `u` from `warm_start[key]` if present, else from the floors. A
  mid-session key change (strategy edit, snapped-duty change, AC↔battery) never re-seeds — it only
  changes which key the next steady window records into. This keeps §2.5's rule that `u` is never
  touched by a transition. The **current** `u` is written to `warm_start[key]` whenever the loop
  has been steady (RPM stdev < 60 over 40 s) — that is what "last settled budget" means.
- **Applied-power back-calculation (against measured draw, not the commanded cap).** After the
  split, guard overrides and the slew clamp, the controller reports the **measured** package and
  GPU power (`Sample.cpu_pkg_w` from RAPL and `Sample.gpu_w` from NVML, both already sampled) to
  `observe_applied`, which back-calculates `u` toward it with `Tt = Ti`. Reporting the commanded
  `cpu_w + gpu_w` instead would be an identity — `split_budget` always commits the whole budget
  — so it would correct nothing. This is the design's only defence against the demand-starved
  wind-up: at idle or in a game lull the temperature error stays positive while nothing consumes
  the budget, and without this the integrator saturates at `cpu_max_w + gpu_max_w` and the next
  load onset runs uncapped through the whole dead time. The old design's conservative start and
  model contour were what covered this; they are deleted, and this replaces them.
  A one-tick draw below the cap is normal, so the back-calculation is applied to a smoothed
  measured draw (the same 5-sample tail-mean the fan window uses) and only while the loop is not
  frozen.

### 2.5 `control/mode.rs` — arbiter (new)

Evaluated every allocator tick, in this order; the first satisfied row wins.

| Mode | Conditions |
|---|---|
| `TempLoop` (Mode A) | `fanctrl` fresh **and** `active` **and** `ec_valid` **and** replica reconciled (no `EC MISMATCH`) **and** argmax controllable **and** T* feasible (2.7) |
| `RpmLoop` (Mode B) | `fan_valid` |
| `Released` | otherwise — nothing to close a loop on. Caps released to stock; flag `FANCTRL LOST` when the socket is absent/stale, `SENSOR LOST` when the fan reading is invalid (both may be set). |

- `active: false` while the socket is fresh means the EC's own curve drives the fans — RpmLoop
  handles that. Socket `absent`/`stale` with a valid fan reading is also RpmLoop, flagged
  `FANCTRL LOST` (informational while in RpmLoop; the flag clears when the socket returns).
- **RpmLoop under `active: false` is expected to have little or no authority, and that is
  reported rather than fought.** The EC's own curve is measured (§Facts) to sit flat at about
  4748 RPM across 67–73 °C, well above any target the user is likely to set, so the loop will
  ask for less heat, drive `u` to its floor, and stay there. The `low` unreachable rule of §2.7
  is exactly what surfaces this: floors held for 60 s with the error still calling for less heat
  raises `TARGET UNREACHABLE (low)` naming the achievable RPM. No extra mechanism is added — the
  point is that the design must not silently sit at the floor pretending to regulate, and the
  applied-power back-calculation (§2.4) is what keeps the integrator from winding while it
  cannot move the plant. Below ~64 °C the same curve is steep, so the run in §5 also checks that
  the loop does not hunt on the steep segment.
- Every transition emits `Effect::Noted { cause: "mode:<from>-><to>:<reason>" }` and a
  `StatusFlag`/telemetry `mode` field. Entering TempLoop re-derives T* and resets `e_{k−1}` —
  the budget `u` is never touched by a transition.
- Hysteresis: a mode must hold its conditions for 3 consecutive ticks (15 s) before the arbiter
  switches **into** it; switching **out** of TempLoop on a hard fault (`absent`, `!active`,
  `!ec_valid`) is immediate. The entry hysteresis is vacuous for RpmLoop, whose only condition
  is continuously true — that is intended, since RpmLoop is the fallback and must be reachable
  at once.
- **The argmax-controllable condition is debounced.** It is evaluated on integer-rounded
  readings, so two sensors within a degree of each other flip the argmax on noise alone; an
  undebounced flip exits TempLoop immediately and then costs 15 s of hysteresis to re-enter,
  which is a self-inflicted mode oscillation. It must therefore fail for 3 consecutive samples,
  or the uncontrollable sensor must lead by more than 1 °C, before it can drop TempLoop. The
  other hard faults stay immediate.
- `Released` in the controller: caps released to stock via the existing `release_to_stock`,
  integrator `Freeze::Released` (u held), flags set as above. When a loop becomes usable again
  the controller re-seeds `u` from the warm-start (or the floors) and re-enters through the
  normal hysteresis — the re-entry is a fresh engagement, not a transition, so the seeding rule
  in §2.4 applies.

### 2.6 Reconciliation (`EC MISMATCH`)

On the first sample that carries a new `print all` view (the view's `observed_at` changed),
compare that sample's replica `max_c` with the view's `temperature` — the poll and the sample
are within one tick of each other and EC temperatures move far slower than that, so no reading
history is kept. Three consecutive views with |Δ| > 1 °C set `EC MISMATCH` (TempLoop unavailable
→ RpmLoop); three consecutive views with |Δ| ≤ 1 °C clear it, and the arbiter's `Decision`
carries `reseed_ma = Some(ma_temperature)` so the controller re-seeds its `EcAverage`. The
comparison also runs on the first view after auto is engaged, before any TempLoop entry.

**The comparison is skipped, not scored, when it cannot be fair.** Both sides are integer
rounded and the view is merged up to one 5 s tick after its capture, while the plant moves at
roughly 0.9 °C/s on a hard load step — enough for a sustained directional skew to spend three
strikes during exactly the ramp the acceptance run requires to stay in TempLoop. So a view is
scored only when the replica's own slope over the last 5 s is below 0.5 °C/s **and** the gap
between the view's `all_observed_at` and the sample's stamp is under 2 s. A skipped view
advances neither counter.

**The moving average is reconciled too.** Comparing only the instantaneous max leaves the
variable the loop actually regulates unchecked, and fw-fanctrl's buffer diverges from a naive
replica in ways that do not show up in the instantaneous value: upstream skips history appends
while paused, and injects a hardcoded 50 °C on any `framework_tool` failure. On every scored
view the controller also checks |`ec_ma` − `ma_temperature`| ≤ 2 °C; three consecutive failures
re-seed the boxcar from `ma_temperature` rather than latching `EC MISMATCH`, since the replica
is recoverable by seeding while the sensor rule is not.

The live `EcAverage` instance is owned by the controller: it pushes each sample's `max_c`,
applies `set_interval(view.ma_interval)` whenever the view changes, re-seeds on request, and
supplies `ec_ma` to the arbiter and to `ControlStatus.ec_ma_c`.

### 2.7 Feasibility and steepness

- **Feasible** iff `T* ≥ max(uncontrollable readings) + 5 °C`. Infeasible ⇒ `TARGET
  UNREACHABLE` flag (existing), mode falls to RpmLoop whose integrator will clamp at the floors;
  the flag text explains why (`T*=61 < ambient 58+5`). The flag clears when feasible again for 60 s.
- **Unreachable from below:** a snapped duty below the curve's lowest tread (the fallback would
  have to snap *up*), or `u` held at its lower bound for ≥ 60 s while the error still calls for
  less heat, raises the same flag with a `low` reason naming the achievable floor RPM. Clears
  after 60 s off the bound.
- **Unreachable from above:** symmetrically, `u` held at `cpu_max_w + gpu_max_w` for ≥ 60 s
  while the error still calls for *more* heat raises the flag with a `high` reason. Without it a
  target above what the fans can reach, or a tachometer that reads valid but stalled, is
  indistinguishable in the telemetry from a converged loop. `Budget` exposes
  `at_upper_bound_for()` alongside `at_lower_bound_for()`, and both feed `ArbiterInput`.
- **Steep** iff `slope_at(T*) > 2 %/°C` ⇒ `STEEP CURVE` flag (warning only; the loop runs). On
  cool16 every tread above 70 °C is steep; on quiet16 none is below 88 °C.

### 2.8 Guards (`control/guards.rs`, new)

Both run every sample in any auto mode, ahead of the arbiter, with hysteresis
(`enter` threshold, `exit = enter − 5 °C`). Both inputs are `Option<f64>`: an absent reading
(dGPU unpowered, NVML N/A, nvme chip missing) makes that guard inactive and exits any hot state.

- **dGPU** (`gpu_hot_c`, default **90**, exit 85; NVML temperature already sampled): while hot,
  the GPU share from `split_budget` is overridden to `max(gpu_floor_w, current gpu_w −
  DOWN_RATE_W)` each 5 s allocator tick; flag `GPU HOT`. The integrator is not frozen — the
  applied-power back-calculation (§2.4) already prevents wind-up against the unusable share.
  **The default is derived, not guessed:** NVML reports this card's
  `GPU Target Temperature Specification` as 87 °C (measured 2026-09-08), which is where its own
  management loop deliberately runs under sustained load. A threshold below that would latch for
  entire gaming sessions and ratchet the GPU to its floor permanently, which is why 83/78 was
  wrong. 90 fires only when the card is hotter than its own loop intends.
- **NVMe** (`nvme_hot_c`, default 80; new hwmon read of the `nvme` chip's `Composite`):
  **reporting only.** While hot it raises the `NVME HOT` flag and the temperature appears in the
  status line and the telemetry. It takes **no control action** — no target boost, no budget
  change. This is a measured decision, not an omission (§Facts, 2026-09-08): under sustained
  reads the drive climbed 67 → 80 °C in 30 s while the fans were already at 4748 RPM, so
  airflow at full tilt did not hold it; and the EC maximum *fell* from 74 to 69 °C over the same
  window, because the load was I/O-bound and left the SoC idle. The guard's only possible lever
  here is to raise the power budget so the SoC heats so fw-fanctrl spins up, and in its own main
  scenario there is nothing to raise. A lever that cannot act when it is needed is worse than an
  honest indicator, and the drive throttles itself regardless.

### 2.9 Actuator read-back

- **CPU.** `set_sustained_mw` writes `--slow-limit=<mw> --stapm-limit=<mw> --fast-limit=<fast>`
  then runs `ryzenadj --info` and parses the table rows `PPT LIMIT SLOW`, `PPT LIMIT FAST`,
  `STAPM LIMIT` (values in W with 3 decimals). Verified iff slow and fast are within 0.5 W of the
  commanded values; STAPM is written but **not** required to verify (reported to fail silently on
  this SoC). Returns `Verified(w)` / `Mismatch { field, commanded, read }` / `Unreadable`.
- **GPU.** No NVML read of the lock exists; verification is the measured SM clock: while
  `gpu_util > 90 %`, `gpu_sm_mhz ≤ locked + 30` for 3 samples (the LUT sweep's pin rule reused).
  Below 90 % utilisation the write is `Unverifiable` (not a failure).
- A candidate `Mismatch` is **re-read once** before it is scored, and is suppressed for 3 ticks
  after an `on_ac` edge. RyzenAdj's own documentation names the platform reasserting vendor
  defaults on AC unplug, power-profile change, and periodic resets as the usual cause of "lost"
  limits, so scoring the first disagreeing read would let ordinary plug-and-unplug reach
  release-to-stock in fifteen seconds. `on_ac` is already on the sample; this is what it is for.
- `Mismatch` (after that re-read) ⇒ `LIMIT-SLIP!` (existing flag), integrator
  `Freeze(ActuatorMismatch)`, immediate reassert on the same tick; three consecutive mismatches
  ⇒ release to stock with the flag held (the existing stickiness watchdog behaviour). **After
  that release the write plus read-back continues every reassert period** rather than stopping:
  otherwise no later `Verified` could ever be produced and auto would be lost until restart.
  The first `Verified` clears the strike count, re-seeds the integrator and re-engages. The RAPL
  stickiness watchdog stays as a second, independent check, unchanged, in `on_sample`.
- `Unreadable` (CPU: `--info` failed, e.g. `ryzen_smu` loaded) and `Unverifiable` (GPU below
  90 % utilisation) are **non-events**: no freeze, no strike. Six consecutive `Unreadable`
  verdicts raise the informational flag `READBACK BLIND`, cleared by the next `Verified`.
- GPU verification runs in the same path: after each NVML lock write the controller feeds
  `verify_lock(gpu_util, gpu_sm_mhz)` and routes its `Mismatch` through the identical
  freeze / flag / reassert / three-strike rule.

## 3. Changes to existing code

### 3.1 Allocator (`control/allocator.rs`)

- `AllocInput { budget_w, demand, floors, cpu_max_w, gpu_max_w, lut }` — `contour`,
  `measured_fan_rpm`, `fan_target_rpm`, `fan_valid`, `fan_slope_rpm_s` are removed.
- `split_budget(budget_w, demand, floors, maxes) -> (cpu_w, gpu_w)`: floors first; the remainder
  is split in proportion to `(demand_cpu, demand_gpu)` (equal split when both are 0), each axis
  capped at its max with the surplus handed to the other axis. Grid `GRID_STEP_W` stays.
- Deleted with their constants: deadband, raise hold, slope gate, overshoot drain, veto, taper,
  `CONSERVATIVE_START`, `overshoot_settle_*`. `UP_RATE_W` / `DOWN_RATE_W` stay as a per-axis slew
  clamp on the split output (safety bound only; the PI's gains set the real pace).
- `allocator::demand` survives unchanged.

### 3.2 Controller (`control/controller.rs`)

`on_auto_sample` becomes: window pushes → guards → (every 5 s) arbiter → error → PI → split →
CPU command + read-back → GPU PI target; the GPU PI itself stays at 1 Hz. The five-gate
adaptation tier, cooldown ring, trust monitor, `ModelSnapshot` period and the degrade guard's
`model.is_none()` are deleted. Auto entry requires only `lut`; `NotCalibrated` means "no LUT".

`PersistedState { lut, calibrated_at, loop_gains: Option<LoopGains>, duty_rpm_table:
DutyRpmTable, warm_start: BTreeMap<String, f64> }` — `model`, `adapt_bias`, `adapt_gain` are
removed (unknown keys in an old file are ignored, as today). Warm-start keys are
`"<strategy>|<duty>|<ac|bat>"`.

`ControlStatus` drops `trim_rpm` and `gain`, gains `mode: LoopMode`, `t_star_c: Option<f64>`,
`ec_ma_c: Option<f64>`, `ec_argmax: Option<String>`, `duty_cmd: Option<u8>`,
`snapped_rpm: f64`, `strategy: Option<String>`, `budget_w: f64`. `StatusFlag` drops
`ModelDistrust`, adds `FanctrlLost`, `EcMismatch`, `SteepCurve`, `GpuHot`, `NvmeHot`,
`ReadbackBlind`. `Effect::ModelSnapshot` is removed. Decision telemetry (`AutoAllocated`)
carries `mode`, `error: f64` (°C or RPM per `mode`), `budget_w`, `freeze`.

The controller's `Budget` is constructed from the persisted gains on auto entry; the GPU
verification verdict is handled like the CPU one (§2.9).

### 3.3 Calibration (`calib/runner.rs`)

Phases: `LutSweep` → `StepTest` → `Done`. `MATRIX_POINTS`, `MatrixPoint`, `Fitting`,
`record_matrix_point` and the `fit_batch` path are deleted.

**Runner ↔ controller interface.** The runner does not read the socket or the arbiter itself.
Each sample the controller hands it a `CalibContext { ec_ma: Option<f64>, ec_mismatch: bool,
fanctrl_active: bool, argmax_controllable: bool, budget_bounds: (f64, f64) }` built from the
same inputs the auto loop uses. The runner requests power through a new
`RunnerEffect::SetBudget(w)`; the controller applies it by freezing the integrator
(`Freeze::Calibrating`), seeding `u = w`, and running the normal `split_budget` → command path
(with read-back) — the calibration never bypasses the caps. `StepTest` therefore consumes
`CalibContext` and produces `SetBudget`; the controller owns both ends of that seam.

`StepTest` gates in two stages, because the obvious ordering self-skips every time. Before the
settle hold it requires only `fanctrl_active` and `!ec_mismatch`. **`argmax_controllable` is
checked after the burner is running, not at the floors:** at idle the machine's own fixture has
`ambient` 47.85 above `apu` 43.85, so the argmax is uncontrollable by construction and an
up-front check would skip the phase before the load that makes it controllable ever starts.
Either gate failing skips the phase with a `Noted` reason and leaves the defaults in force.

1. Start the CPU burner, then hold `u` at the floors until the EC moving average is flat
   (≤ 0.5 °C change over 60 s) and RPM steady; cap 5 min. The burner starts **before** the
   settle so the idle-to-load transition is inside the baseline rather than folded into the
   step's ΔT.
2. Step `u` by `STEP_W = 30` W split by demand (GPU load is user-provided as today — nag
   `NeedsLoad` if GPU utilisation stays low), hold 5 min or until the EC average has been flat
   for 90 s. Abort the step, restore the floors and skip with a `Noted` reason if the EC max
   exceeds 95 °C at any point.
3. Fit a first-order-plus-dead-time model by least squares on the step response, separately for
   the EC average (`k_c_per_w`, `tau_s`, `theta_s`) and for RPM (`k_rpm_per_w`, `tau_rpm`,
   `theta_rpm`). **The gain's denominator is the measured applied-power delta per axis, not
   `STEP_W`:** `split_budget` divides the 30 W between CPU and GPU by demand and the draw need
   not equal the cap, so dividing by 30 would bias `K` low and therefore `Kc` high. The runner
   records the smoothed measured draw before and after the step and passes that delta. The fit
   is aborted (skip, `Noted`) if the argmax label changes mid-step, since a sensor handover
   shows up as spurious dead time.
   Derive `λ = max(90, 3·θ_eff)` where `θ_eff = θ + ma_interval/2` for the EC fit (the regulated
   variable is the boxcar mean, §2.4), `Kc = τ / (K·(λ + θ_eff))`, `Ti = τ` for each.
4. Persist `LoopGains` with `fitted_at` (stamped by the runner from the sample clock). A fit is
   rejected (defaults kept, `Noted` reason) if `K ≤ 0`, `τ < 5 s`, the applied power never rose
   (no load), the response magnitude was too small to identify (< 3 °C for the EC fit, < 150 RPM
   for the RPM fit), or the derived `Kc` falls outside [0.25, 4]× the corresponding default.
   The magnitude and sanity bounds are what stop the unbounded `Kc = τ/(K(λ+θ))` as `K → 0⁺`:
   a step that stays under quiet16's 55 °C knee leaves the duty pinned at 15, fits a tiny
   positive `k_rpm_per_w`, and would otherwise persist a Mode B gain orders of magnitude too
   large — into `state.json`, where it is loaded on every subsequent start.

The runner starts the existing CPU burner for the step and nags `NeedsLoad` while GPU
utilisation stays below the LUT-sweep pin threshold, exactly as the matrix phase did. Fans are
never commanded during calibration. `Freeze::Calibrating` is asserted for the **whole**
calibration session — the LUT sweep keeps its existing direct actuator path while the integrator
is frozen, and `u` is unchanged from calibration start to exit; the step is applied through the
normal caps.

### 3.4 Sensors, sampler, config

- `sensors/ec.rs` (2.2) and an `nvme_temp_c` reader added to the sampler; `Sample` gains
  `ec: Option<EcReading>`, `ec_valid`, `nvme_temp_c: Option<f64>`, `fanctrl: Option<FanctrlView>`,
  `on_ac: bool` (from `/sys/class/power_supply/ACAD/online`). `cpu_pkg_w` (RAPL) and `gpu_w`
  (NVML) are already sampled and now also feed `Budget::observe_applied` (§2.4); the existing
  `resumed` signal now additionally invalidates the EC boxcar and the steady-window state.
- Config keys added: `fanctrl_socket` (path), `gpu_hot_c`, `nvme_hot_c`.
  Unknown keys stay tolerated; the `online_rls`-specific legacy note and test in `config.rs`
  are generalised to "unknown keys are ignored".

### 3.5 UI, telemetry, docs

- TUI header: `mode A|B|rel · T* 71.5°C · ma 70.8 · duty 31 → 3050 rpm · budget 68.0 W`
  replaces the trim/gain segment; new flags rendered with severities (`GPU HOT` and `NVME HOT`
  warning, `EC MISMATCH` and `FANCTRL LOST` warning, `STEEP CURVE` info). `k` still starts the
  calibration; its progress shows `lut` / `step` phases.
- Telemetry `sample` line adds `ec_max`, `ec_argmax`, `ec_ma`, `nvme_c`, `fanctrl_speed`,
  `fanctrl_active`, `strategy`; `decision` lines add `mode`, `t_star`, `budget_w`, `freeze`;
  `trim_rpm`, `gain`, `model_*` are removed.
- README: rewrite **Calibration walkthrough** (LUT sweep + step test, ~10 min), **Auto mode**
  (the cascade, the two modes, the flags table), **Safety model** (fw-fanctrl still owns the
  fans; the controller only reads the socket; guards; read-back), and the **Configuration** table
  (drop `online_rls`, add the new keys). `docs/research/03-control.md` gets a pointer note to 05.

## 4. Deletions (must sweep the whole repo, source and non-source)

`src/control/thermal_model.rs`, `src/control/kalman.rs`, `src/control/trust.rs`,
`src/control/cooldown.rs`, `src/control/trim.rs` (test-only, used only by the deleted sims), the
`MATRIX_POINTS`/`MatrixPoint`/`Fitting` code in `calib/runner.rs`, the adaptation tier and its
tests in `controller.rs`, the three field-replay sims in `allocator.rs` (replaced by the
emulator-based sim, §5), `PersistedState.model/adapt_bias/adapt_gain`, `Effect::ModelSnapshot`,
`ControlStatus.trim_rpm/gain`, `StatusFlag::ModelDistrust`, telemetry `trim_rpm/gain/model_*`,
README mentions of the model, matrix and trim. Search for each deleted path **and** its basename
across the repo, including `docs/`, `TODO.md` and any manifest.

## 5. Testing

- Unit tests per new module: curve inversion on the live `quiet16` and `cool16` curves (treads,
  T*, slopes, unreachable integers), the replica's rounding and drop rules against the recorded
  hwmon/socket pair (47.85 → 48), the boxcar off-by-one, `split_budget` floors/caps/surplus,
  PI velocity form + anti-windup + freeze, arbiter table and hysteresis, table refinement,
  `ryzenadj --info` parsing on a captured fixture, FOPDT fit on a synthetic step.
- **fw-fanctrl emulator** (`test_support`): 1 s tick, boxcar of N non-zero samples with the
  off-by-one, `eff = min(MA, current)`, file-order piecewise-linear curve, `int()` truncation,
  `active` and strategy switchable mid-run. Chained: watts → EC temperature (first order, τ 35 s,
  θ 20 s, K 0.8 °C/W, plus an ambient offset) → emulator → duty → RPM via the seeded table with a
  one-sided momentum kick on positive slew (the 2026-07-14 finding) and ±90 RPM noise.
- Closed-loop acceptance on the chained plant: after a load step, ≥ 90 % of a 30 min window inside
  ±150 RPM, and no sustained relay cycle. **The relay test is period-agnostic**, because a window
  pinned to 60–200 s would miss the loop's own worst case: with `Ti = τ` the nominal loop is
  integrating-plus-dead-time with `θ_eff ≈ 50 s`, whose ultimate period is about `4·θ_eff ≈
  200 s` — the old window's upper edge exactly. The criterion is instead: no ≥ 3 consecutive
  band excursions alternating in sign within the run, at any period, with the excursion count
  and the dominant period reported. Run once per active strategy (`quiet16`, `cool16`) and once
  in each mode.
- Suspend/resume: a scripted `resumed` sample mid-run clears the boxcar, the fan window and the
  steady-window state, and no warm-start entry or table refinement is written from a window that
  straddles the gap.
- Bumpless tests: socket death mid-run (A→B), `active: false` mid-run with a fresh socket (A→B),
  and a strategy edit mid-run leave `u` continuous (|Δu| ≤ one PI step) and the caps continuous.
  A `Released` run (socket absent **and** fan reading invalid) shows stock caps, both flags, and
  a re-engagement into RpmLoop from the warm-start without a cap step.
- Robustness: the acceptance configurations are re-run with the plant's K, τ, θ perturbed ±50 %
  and with the plant's duty→RPM table biased −8 % from the controller's seed; in the biased run,
  passive refinement brings measured RPM inside ±150 within 20 min and T* follows the re-snapped
  duty. The plant's table is therefore a separate object from the controller's seed.
- The acceptance sims run against the final controller (warm-start, refinement and calibration
  hooks included) — they are the last leaf before the sweep, not a mid-tree check.
- Machine fixtures (captured once, checked in under `tests/fixtures/`): `print all` for
  `quiet16` and `cool16`, `print speed`, two cros_ec hwmon trees (dGPU unpowered and dGPU
  powered, the latter paired with the `print all` reply taken at the same moment), nvme
  `Composite`, `ACAD/online`, a `ryzenadj --info` table, and today's `state.json` with
  `model`/`adapt_*`. ENODATA is represented by a present `temp*_label` with **no** `temp*_input`
  file, so the production reader's ordinary "read failed ⇒ drop" branch covers it. All
  cfg(test) support lives under `src/test_support/{mod,fixtures,fakes,plant}.rs`.
- All timing decisions (freshness, hysteresis, steady windows, clears) are computed from the
  monotonic timestamps carried on `Sample`/`FanctrlView`, never from a wall clock read inside a
  decision path, so every duration-shaped criterion is drivable from scripted samples. The
  poller thread uses its own `Instant` only for its cadence.
- The plant is driven by a hand-rolled seeded xorshift (no new dependency). It also emits
  scripted `cpu_util`/`gpu_util`/`gpu_sm_mhz` tracks so the demand split, the GPU PI and
  `verify_lock` see a real load step, and scripted `gpu_*` EC channels so the powered-dGPU
  case can be run.
- Additional graded runs: the calibration StepTest executed on the plant, with the derived
  gains then loaded for a repeat of the quiet16/TempLoop acceptance; a load-release transient
  (the `min(MA, current)` branch) that must stay inside ±150 within 90 s; a dGPU-powered-and-hot
  30 min run that must stay in TempLoop; a dGPU-unpowered run; a sub-floor target run that must
  raise `TARGET UNREACHABLE (low)` and hold the floor without relay; a three-strike mismatch run;
  a global assertion that only `print speed`/`print all` were ever sent; and an assertion that
  `ec_ma_c` tracks the emulator's `ma_temperature` within 1 °C in steady state and that at least
  one steady window is detected per converged run.
- Feasibility (`T*` below ambient + 5), steep curve, read-back mismatch (freeze + flag), `GPU HOT`
  (GPU share decays, budget unchanged), `NVME HOT` (flag only — the commanded caps and the RPM
  trace must be identical to the same run with the drive cool), reconciliation
  (three mismatches → B, three matches → A with re-seeded average).
- The existing `Runner`/`FakeRunner` and `GpuClockCtl`/`FakeGpu` seams stay the only test seams;
  the socket client gets a `FanctrlSource` trait with a fake.

## 6. Follow-ons (not in this tree)

- Gain scheduling of `Kc` on commanded duty (power-law 0.8, clamp [0.5, 2]×).
- A slow outer RPM trim on T* (λ 300–600 s, ±1 tread authority) if passive table refinement
  proves insufficient in the field.
- RyzenAdj `--tctl-temp` as a hardware CPU leg.
- Upstream note to fw-fanctrl about the 0777 socket + `set_config`-as-root.
- **NVMe cooling, if it ever proves worth acting on.** The guard is reporting-only because the
  2026-09-08 probe found near-maximum airflow did not hold the drive and the SoC was idle while
  it heated (§2.8). A future design with a real lever would need a clean steady-state
  measurement of drive temperature at low versus high fan speed, which this run did not get, and
  a mechanism that does not depend on the SoC drawing power.
- **Field validation (post-merge, user-owned):** the goal's 30 minute gaming-session criterion
  is graded on real telemetry after deployment, following this project's pending-👤-validation
  pattern. The tree stops at merge-ready; the in-tree evidence is the chained-plant acceptance
  with perturbed plant parameters.

## Task tree (no-beads mode; root slug `fwloop`)

All rows are depth 1 under the root. `deps` lists blocker row ids. Every dep is paired with a
`blocked-by` line in the description. `owns:`/`consumes:` name the boundaries a task exchanges
with siblings. Acceptance criteria that depend on another task cite it as `(needs: <id>)`.
`promotion` records the promotion-review verdict once applied. Rows amended by coverage rounds
carry `[cov-1]`/`[cov-2]`; the ledger is `docs/superpowers/reviews/fwloop-coverage-ledger.md`.
Every controller edit outside fwloop.12/19/22 is confined to the named call site(s) so the
hot-file cap serialises them without conflict.

| id | depth | deps | title | description | promotion |
|---|---|---|---|---|---|
| fwloop.20 | 1 | — | Capture machine fixtures + test-support layout | `tests/fixtures/`: `fanctrl/print_all_quiet16.json`, `fanctrl/print_all_cool16.json` (verbatim live `print all` replies with each strategy resolved), `fanctrl/print_speed.json`, `hwmon/cros_ec_idle/` (`name` + `temp*_label`/`temp*_input` with ambient 47850, charger 44850, apu 43850, cpu@4c 40850, three gpu −150, and `gpu_temp@40` as a **label with no `_input` file** — the ENODATA convention), `hwmon/cros_ec_load/` (the recorded loaded set: cpu@4c 74850, ambient 69850, apu 69850, charger 63850) paired with `fanctrl/print_all_load.json` (`temperature: 75.0`) captured in the same second, `hwmon/cros_ec_dgpu_on/` (**dGPU powered at 18.9 W and the `gpu_*` sensors still −150/absent** — this fixture pins the measured fact that they never report, replacing the round-2 assumption that they come alive), `hwmon/nvme/`, `power_supply/ACAD/online`, `ryzenadj_info.txt` (captured with `ryzen_smu` unloaded, via the sudo pattern), `state_v1.json` (today's `state.json` with `model`/`adapt_*`). Test-support module layout `src/test_support/{mod,fixtures,fakes,plant}.rs` (cfg(test)), `fixtures::path(...)`. **owns:** the fixture corpus, the ENODATA-as-missing-input convention, the test-support module layout. Files: `tests/fixtures/**`, `src/test_support/mod.rs`, `src/test_support/fixtures.rs`, `src/main.rs` (cfg(test) mod decl). Acceptance: every file above exists and is committed; `fixtures::path` resolves each; the recorded cool16 curve reproduces `int(duty_at(51.8)) == 21` and a tread above 70 °C with slope > 2 %/°C; the loaded tree's rounded max (75) equals its paired `print all` `temperature`; the dGPU-on tree contains no positive `gpu_*` reading. | [cov-1] GAP fixtures · [cov-2] · [roast-1] |
| fwloop.1 | 1 | — | Curve model + DutyRpmTable | New `src/fanctrl/{mod,curve,table}.rs`. `Curve::from_points(Vec<(f64, u8)>) -> Result<Curve>` in file order, **rejecting any descending segment** (§2.1: a non-monotone curve makes `tread` ill-posed and hides steepness); `duty_at`, `tread`, `t_star`, `slope_at`, `nearest_tread(d) -> Option<u8>` (the §2.3 fallback: nearest duty with a tread, preferring lower; `None` when no tread exists at or below `d`), `min_tread_duty()`. `DutyRpmTable` with `Default` = the ten seeded points (also the serde default), `duty_for_rpm` (ties down), `rpm_for_duty`, `refine(duty, mean_rpm)` EWMA 0.8/0.2 rejected when `|mean − rpm| > 25 %` and clamped to keep the table strictly increasing in duty. Unit tests use the §Facts point lists directly (no fixture dependency). **owns:** `Curve` API including `nearest_tread`/`min_tread_duty`, `DutyRpmTable` API, its `Default` and serde shape, the refinement invariants. Files: `src/fanctrl/curve.rs`, `src/fanctrl/table.rs`, `src/fanctrl/mod.rs`, `src/main.rs` (mod decl). Acceptance: on the `quiet16`/`cool16` points — treads, T*, slopes, `nearest_tread` for a skipped integer resolves to the nearest lower tread and to `None` below the floor; a curve with a descending segment is rejected; `default()` equals the seed and a JSON without the field deserialises to it; interpolation, snap ties; a refinement that would invert two adjacent duties is clamped and the table stays monotone after 100 noisy refinements; a > 25 % jump is rejected. | [cov-1] · [cov-2] |
| fwloop.2 | 1 | fwloop.20 | fw-fanctrl socket client | `src/fanctrl/client.rs`: `PrintCommand { Speed, All }` (the only commands the client can send), `FanctrlSource` trait + `UnixFanctrlClient` (connect 1 s, read 3 s, raw CLI string, read to EOF) + `FakeFanctrl` in `src/test_support/fakes.rs` that records every command and replays scripted views/failures. `FanctrlView` (§2.1) with `observed_at` (any poll) and `all_observed_at` (last `print all`), `curve` as the raw `Vec<(f64, u8)>` from `resolve_curve(print_all_json, strategy)` (exact-name match in `strategies`); `Freshness { Fresh, Stale, Absent }` with the 15 s (`print speed`) / 90 s (`print all`) rules. Config key `fanctrl_socket` (default `/run/fw-fanctrl/.fw-fanctrl.commands.sock`). **owns:** `PrintCommand`, `FanctrlSource` trait, `FanctrlView` struct and its two stamps, `resolve_curve` (strategy resolution), `Freshness` and its timing rules, `fanctrl_socket` config key, `FakeFanctrl`. **consumes:** socket fixtures and the fakes module slot (fwloop.20). blocked-by fwloop.20: consumes the `print all`/`print speed` fixtures and `src/test_support/fakes.rs`. Files: `src/fanctrl/client.rs`, `src/fanctrl/mod.rs` (mod decl only), `src/config.rs`, `src/test_support/fakes.rs`. Acceptance: parses the fixtures (strategy, active, speed, temperature, ma interval); `resolve_curve` yields exactly the §Facts points for `quiet16` and `cool16`; ENOENT → Absent; read timeout → Stale; `print speed` failing for 15 s while `print all` is fresh → Stale; a `print speed` refresh bumps `observed_at` but not `all_observed_at`; the fake's command log contains only `Speed`/`All`. | [cov-1] · [cov-2] |
| fwloop.3 | 1 | fwloop.20 | EC replica + NVMe + AC sensors | `src/sensors/ec.rs`: `EcReading`/`EcLabel` (controllable `apu`, `cpu`, `gpu_*` if one ever reports; uncontrollable `ambient`, `charger`), every positive reading takes part in the max, drop ≤0 / unreadable / unparsable `_input`, round to integer, max + argmax (ties: sysfs order); `EcAverage` boxcar of N non-zero samples with the off-by-one, `set_interval(n)` cap 100 **retaining** existing samples, `reseed(value)` the only clearing operation, plus `is_seeded()`/`sample_count()` so the controller can refuse to use an underfilled mean as a full one. `sensors/hwmon.rs` gains `nvme_composite_c() -> Option<f64>` and `on_ac()`. **owns:** `EcReading`, `EcLabel` and its controllability rule, `EcAverage` and its retain-on-resize contract, `nvme_composite_c`, `on_ac`. **consumes:** hwmon/power_supply fixtures (fwloop.20). blocked-by fwloop.20: consumes the two cros_ec trees, nvme and ACAD fixtures. Files: `src/sensors/ec.rs`, `src/sensors/hwmon.rs`, `src/sensors/mod.rs`. Acceptance: on `cros_ec_idle` 47.85 → 48, argmax `ambient`, the −150 and the input-less sensor dropped without invalidating the reading; on `cros_ec_load` 74.85 → 75 with argmax `cpu@4c`, matching that fixture's paired socket `temperature` of 75.0; on `cros_ec_dgpu_on` the `gpu_*` sensors are still −150/absent and are dropped, so the max comes from `cpu`/`ambient` (measured: they never report even with the dGPU powered); a synthetic positive `gpu_*` reading would join the max and classify controllable; boxcar returns mean of n−N..n−1; `set_interval` grows/shrinks without clearing, `reseed` replaces the contents; `is_seeded` is false until seeded or N samples deep; nvme/ac readers on fixtures, nvme `None` when the chip is absent. | [cov-1] · [cov-2] · [roast-1] |
| fwloop.4 | 1 | — | Budget integrator | `src/control/budget.rs`: `Budget::new(&LoopGains)`, `set_gains`, velocity-form PI, `PI_PERIOD_S=5`, `LoopError { Temp{e_c}, Rpm{e_rpm} }`, `Freeze { ActuatorMismatch, Calibrating, Released }`, clamp + back-calculation `Tt=Ti` against the bounds **and** `observe_applied(measured_w)` against the **measured** smoothed draw (§2.4 — not the commanded cap, which would be an identity), `set_bounds(lo,hi)`, `seed(u)`, `resync_error(e)` (resets e_{k−1} without touching u), `step(err, freeze) -> f64`, an error-kind switch calls `resync_error` implicitly; `at_lower_bound_for()` and `at_upper_bound_for() -> Duration` for the two unreachable rules; `scale_rpm_gain(slope_at_t_star)` applying the §2.4 curve-slope schedule. `LoopGains` struct with serde and `Default` = the θ_eff-derived IMC defaults of §2.4 (`kc_w_per_c = 0.22`, `ti_s = 35`). `WarmStart` map API: `key(strategy,duty,on_ac) -> String`, `lookup`, `record`. **owns:** `Budget`, `LoopError`, `Freeze`, `LoopGains`, `WarmStart` types. Files: `src/control/budget.rs`, `src/control/mod.rs`. Acceptance: step response on a first-order plant reaches within 1 % with overshoot ≤ 5 % at defaults; a non-default `LoopGains` changes the step magnitude; clamp holds at bounds without wind-up (release recovers within one Ti); **a measured draw held below the commanded cap stops `u` growing and pulls it back toward the draw** (the demand-starved case — the roast's Blocking finding); freeze holds u exactly; `resync_error` after a setpoint jump produces no proportional kick; a Temp→Rpm switch produces |Δu| ≤ one integral increment; `at_lower_bound_for`/`at_upper_bound_for` count only while clamped at their own bound; `scale_rpm_gain` returns 1× at `slope_ref` and 0.25× at four times `slope_ref`. | [cov-1] · [cov-2] · [roast-1] |
| fwloop.5 | 1 | — | Allocator: scalar budget split | `src/control/allocator.rs`: `AllocInput { budget_w, demand, floors, cpu_max_w, gpu_max_w, gpu_floor_w }`; `split_budget` (floors first, ∝ demand, surplus to the other axis, `GRID_STEP_W`); `UP_RATE_W`/`DOWN_RATE_W` slew clamp retained; delete deadband/raise-hold/slope-gate/overshoot drain/veto/taper/`CONSERVATIVE_START`/`overshoot_settle_*`, their constants, and the three field-replay sims (`simulate_field_cycle`, `simulate_soak_cycle`, `simulate_ec_overshoot_cycle`) — `control/trim.rs` becomes unused and is deleted by fwloop.14. `allocator::demand` unchanged. The controller edit is confined to the `allocator.step` call site, compiling against the new shape with a placeholder budget (sum of floors) until fwloop.12. **owns:** `AllocInput` shape, `split_budget`. Files: `src/control/allocator.rs`, `src/control/mod.rs`, `src/control/controller.rs` (the `allocator.step` call site only). Acceptance: floors always met; both axes capped with surplus reassigned; equal split at zero demand; slew clamp bounds per-tick change; no reference to `contour`, `CONSERVATIVE_START` or `overshoot_settle` remains under `src/control/allocator.rs` or the call site (the repo-wide sweep is fwloop.14's). | [cov-1] · [cov-2] |
| fwloop.6 | 1 | — | Guards (dGPU, NVMe) + config keys | `src/control/guards.rs`: `Guards::step(gpu_temp_c: Option<f64>, nvme_temp_c: Option<f64>) -> GuardState { gpu_hot, nvme_hot }` with enter thresholds, exit = enter − 5, and `None` ⇒ that guard inactive (exits any hot state); `gpu_share_override(current_gpu_w, gpu_floor_w)`. **There is no `effective_target`:** the NVMe guard is reporting-only (§2.8, measured), so nothing modifies the user's RPM target and `nvme_hot` only drives a flag. Config keys `gpu_hot_c` (**90**, exit 85 — derived from the card's own 87 °C target, §2.8), `nvme_hot_c` (80); the `online_rls` legacy note/test in `config.rs` generalised to "unknown keys are ignored". **owns:** `Guards`, `GuardState`, the absent-reading rule, the two config keys, the unknown-key tolerance rule. Files: `src/control/guards.rs`, `src/config.rs`, `src/control/mod.rs`. Acceptance: hysteresis enters at threshold, exits 5 below; a `None` input deactivates the guard and clears a hot state; the GPU share override is computed per spec §2.8; a hot NVMe sets `nvme_hot` and changes nothing else (no target, no budget); config round-trips with defaults; a config containing `online_rls`, a stale `nvme_boost_rpm` and an arbitrary unknown key still loads. | [cov-2] · [roast-1] |
| fwloop.7 | 1 | fwloop.20 | Actuator read-back | `src/actuators/cpu.rs`: write `--slow-limit --stapm-limit --fast-limit`, then run `ryzenadj --info` through the `Runner`, parse `PPT LIMIT SLOW`/`PPT LIMIT FAST`/`STAPM LIMIT` rows; return `WriteVerdict { Verified(w), Mismatch{field,commanded,read}, Unreadable, Unverifiable }` (slow/fast within 0.5 W; STAPM not required; a failed `--info` ⇒ `Unreadable`, never `Mismatch`). `src/actuators/gpu.rs`: `verify_lock(gpu_util, gpu_sm_mhz) -> WriteVerdict` using the LUT-sweep pin rule (util > 90 %, sm ≤ locked + 30, 3 samples). Existing controller callers compile by treating non-`Verified` as today's success/failure until fwloop.12; the controller edit is confined to the two actuator call sites; the RAPL stickiness watchdog in `on_sample` is untouched. **owns:** `WriteVerdict`, `set_sustained_mw` return type, `verify_lock`. **consumes:** the `ryzenadj --info` fixture (fwloop.20). blocked-by fwloop.20: consumes `ryzenadj_info.txt`. Files: `src/actuators/cpu.rs`, `src/actuators/gpu.rs`, `src/control/controller.rs` (the two call sites only). Acceptance: parser on the fixture table; mismatch detected on a fake runner returning a stale table; a runner whose `--info` fails yields `Unreadable`; GPU verdict `Unverifiable` under 90 % util and `Mismatch` when the pinned clock exceeds lock + 30 for 3 samples; the existing RAPL watchdog tests still pass. | [cov-1] · [cov-2] |
| fwloop.8 | 1 | — | Controller status surface | `src/control/controller.rs` types only: `LoopMode { TempLoop, RpmLoop, Released }`; `ControlStatus` drops `trim_rpm`/`gain`, adds `mode`, `t_star_c: Option<f64>`, `ec_ma_c: Option<f64>`, `ec_argmax: Option<String>`, `duty_cmd: Option<u8>`, `snapped_rpm: f64`, `strategy: Option<String>`, `budget_w: f64`; `CalibProgressLite.phase` stays a plain `String`; `StatusFlag` drops `ModelDistrust`, adds `FanctrlLost`, `EcMismatch`, `SteepCurve`, `GpuHot`, `NvmeHot`, `ReadbackBlind` with severities; `Effect::ModelSnapshot` removed; `Effect::AutoAllocated` gains `mode`, `error: f64`, `budget_w`, `freeze: Option<&'static str>` (all plain types — this task stays dependency-free). Existing code populates the new fields with defaults so the crate compiles; UI/telemetry call sites updated minimally (compile-only). **owns:** `LoopMode`, `ControlStatus` field set, `StatusFlag` set, `Effect` variants. Files: `src/control/controller.rs`, `src/ui/view.rs` (compile-only), `src/telemetry.rs` (compile-only). Acceptance: crate compiles and tests pass with the new type surface; `flag_severity` covers every new flag; the new `ControlStatus`/`Effect` definitions carry no `trim_rpm`/`gain`/`ModelSnapshot` field or variant. The repo-wide symbol sweep is **not** this task's — `controller.rs` still holds the adaptation tier and its tests at this point, and removing them is fwloop.22's chartered work; this task only reshapes the type surface and updates call sites enough to compile. | [cov-1] · [cov-2] |
| fwloop.22 | 1 | fwloop.8 | Remove the adaptation tier | `src/control/controller.rs`: delete the five-gate adaptation tier, the cooldown ring, the trust monitor, the `ModelSnapshot` period, the degrade guard's `model.is_none()` check, `persisted_bias`/`persisted_gain` plumbing, and every controller test that exercises them (the KF/trust/cooldown/`fitted_model` blocks). The auto path compiles with the budget stubbed at the floors and the existing `AllocInput` call site; the crate builds and the surviving tests pass. **owns:** the removal of the adaptation tier and its tests. **consumes:** the `ControlStatus`/`Effect` surface without `trim_rpm`/`gain`/`ModelSnapshot` (fwloop.8). blocked-by fwloop.8: consumes the model-free status surface. Files: `src/control/controller.rs`. Acceptance: no import of `thermal_model`, `kalman`, `trust`, `cooldown` remains in the controller (source or tests); no `adapt_bias`/`adapt_gain`/`persisted_bias` symbol remains; `cargo test` green. | [cov-2] split from fwloop.12 |
| fwloop.9 | 1 | fwloop.2, fwloop.3 | Sample plumbing + socket poller | `src/types.rs` `Sample` gains `ec: Option<EcReading>`, `ec_valid`, `nvme_temp_c: Option<f64>`, `fanctrl: Option<FanctrlView>`, `fanctrl_freshness`, `fanctrl_view_changed: bool` (true on the first sample whose view `all_observed_at` differs from the previous sample's — a speed-only refresh never sets it), `on_ac`, and `resumed` (already produced by the existing resume handler, now also consumed by the EC boxcar and steady-window invalidation of §2.2). `src/sensors/sampler.rs` (1 Hz tick) reads EC/NVMe/AC each tick and merges the latest view from a background `FanctrlPoller` thread (`print speed` every 5 s, `print all` every 30 s, never faster; shares an `Arc<Mutex<...>>`); `src/main.rs`: the poller is constructed from `Config::fanctrl_socket` with the real `UnixFanctrlClient` and its thread is joined on shutdown. All freshness/cadence decisions are computed from the `Sample`/`FanctrlView` monotonic timestamps; the poller's own `Instant` drives only its cadence. **owns:** the new `Sample` fields including `fanctrl_view_changed`, `FanctrlPoller` cadence, poller construction and shutdown, the timestamp-not-wall-clock rule. **consumes:** `EcReading`/`nvme_composite_c`/`on_ac` (fwloop.3), `PrintCommand`/`FanctrlSource`/`FanctrlView`/`Freshness` (fwloop.2). blocked-by fwloop.2: consumes `FanctrlSource` trait, `PrintCommand`, `FanctrlView` (two stamps), `Freshness`. blocked-by fwloop.3: consumes `EcReading`, `nvme_composite_c`, `on_ac`. Files: `src/types.rs`, `src/sensors/sampler.rs`, `src/sensors/mod.rs`, `src/main.rs`. Acceptance: with the fake source, over a 60 s scripted run the poller issues `Speed` at 5 s ±1 tick and `All` exactly twice; `Stale` after 90 s of `All` failures and after 15 s of `Speed` failures; `fanctrl_view_changed` is true exactly once per new `All` view and never on a `Speed`-only refresh; the real client is constructed from the configured path and the thread joins on shutdown. | [cov-1] · [cov-2] |
| fwloop.10 | 1 | fwloop.1, fwloop.2, fwloop.3, fwloop.8 | Mode arbiter, reconciliation, feasibility | `src/control/mode.rs`: `Arbiter::decide(&ArbiterInput) -> Decision { mode, t_star, slope, reasons, flags, reseed_ma: Option<f64>, ec_mismatch: bool, t_star_changed: bool }` implementing the §2.5 table, 3-tick entry hysteresis, immediate exit on hard faults, `EC MISMATCH` counters (§2.6; `view_changed: bool` in the input is a plain flag supplied by the controller) **including the skip rule** (a view is scored only when the replica's 5 s slope is under 0.5 °C/s and the view-to-sample stamp gap is under 2 s) **and the moving-average check** (|`ec_ma` − `ma_temperature`| ≤ 2 °C, three failures requesting a re-seed rather than latching a mismatch), feasibility + steepness (§2.7) with the 60 s feasible-again clear, the **low** unreachable rule (`target_duty` below `min_tread_duty()`, or `at_lower_bound_for ≥ 60 s` with the error still negative), the symmetric **high** rule (`at_upper_bound_for ≥ 60 s` with the error still positive), the **debounced** argmax-controllable condition (3 consecutive failures or a > 1 °C lead before it can drop TempLoop), `FANCTRL LOST` clearing on the first fresh view, and re-derivation of T* whenever the view's curve **points** differ from the cached ones (cache keyed on points, not the strategy name) or `target_duty` changes — `t_star_changed` tells the controller to `resync_error`. `ArbiterInput` carries `fanctrl: Option<&FanctrlView>` + `Freshness` + `view_changed`, `ec: Option<&EcReading>`, `ec_ma: Option<f64>`, `fan_valid`, `target_duty`, `at_lower_bound_for`, `error_sign`. **owns:** `Arbiter`, `ArbiterInput`, `Decision`, mismatch/feasibility counters, the point-keyed cached `Curve`, all T* derivation. **consumes:** `Curve::from_points/tread/t_star/slope_at/min_tread_duty` (fwloop.1), `FanctrlView`/`Freshness` (fwloop.2), `EcReading`/`EcLabel` (fwloop.3), `LoopMode`/new `StatusFlag`s (fwloop.8). blocked-by fwloop.1: consumes `Curve::from_points`, `tread`, `slope_at`, `min_tread_duty`. blocked-by fwloop.2: consumes `FanctrlView` + `Freshness`. blocked-by fwloop.3: consumes `EcReading`, `EcLabel` controllability. blocked-by fwloop.8: consumes `LoopMode`, `StatusFlag` variants. Files: `src/control/mode.rs`, `src/control/mod.rs`. Acceptance: table-driven tests for every row of §2.5; 3-tick hysteresis in, immediate out; three mismatches → RpmLoop, three matches → TempLoop with `reseed_ma`; infeasible target yields `TargetUnreachable` with the explanatory text and clears only after 60 s of continuous feasibility; a sub-floor duty and a 60 s low-clamp each yield the `low` reason; `FanctrlLost` clears on the first fresh view; a cool16 tread above 70 → `SteepCurve`; an edit of the active strategy's points under the same name re-derives T* and sets `t_star_changed`. | [cov-1] · [cov-2] |
| fwloop.11 | 1 | fwloop.1, fwloop.4, fwloop.20 | Persisted state migration | `src/state.rs`: `PersistedState { lut, calibrated_at, loop_gains: Option<LoopGains>, duty_rpm_table: DutyRpmTable, warm_start: BTreeMap<String,f64> }`; remove `model`, `adapt_bias`, `adapt_gain` and the `state.rs` thermal-model fit tests; old files load (unknown keys ignored, missing new keys default). The controller edit is confined to the persist call sites (`save_persisted_state`, `exit_auto_and_persist`, `apply_calib_effects`). **owns:** the `state.json` schema. **consumes:** `LoopGains`/`WarmStart` (fwloop.4), `DutyRpmTable` serde + `Default` (fwloop.1), `state_v1.json` (fwloop.20). blocked-by fwloop.1: consumes `DutyRpmTable` serde shape and `Default`. blocked-by fwloop.4: consumes `LoopGains` serde shape. blocked-by fwloop.20: consumes the `state_v1.json` fixture. Files: `src/state.rs`, `src/control/controller.rs` (persist call sites only). Acceptance: the `state_v1.json` fixture loads with `lut` intact, `duty_rpm_table` equal to the ten seeded points, `warm_start` empty and `loop_gains` `None`; round-trip of the new schema; no `thermal_model` import remains in `state.rs`. | [cov-1] · [cov-2] |
| fwloop.12 | 1 | fwloop.1, fwloop.3, fwloop.4, fwloop.5, fwloop.6, fwloop.7, fwloop.8, fwloop.9, fwloop.10, fwloop.11, fwloop.22 | Controller loop integration | `src/control/controller.rs`, on the tier-free controller from fwloop.22: `Budget::new(persisted loop_gains or default)` on auto entry; `on_auto_sample` = window pushes (existing fan window; `rpm_smoothed` = `FAN_SMOOTH_N` tail-mean, `fan_valid` from the sample) + `EcAverage` push (controller owns the live instance: `set_interval` on view change, `reseed` on `Decision.reseed_ma`) → guards (`Option` inputs; `nvme_hot` raises its flag only) → every 5 s: budget bounds (`lo = cpu_floor_w + lut.watts_at(gpu_floor_mhz)`, `hi = cpu_max_w + gpu_max_w`, `Budget::set_bounds`) → `target_duty` from the user's target via `DutyRpmTable::duty_for_rpm` + `Curve::nearest_tread` → arbiter (`ArbiterInput` incl. `view_changed`, `at_lower_bound_for`, `at_upper_bound_for`, `error_sign`) → `resync_error` when `t_star_changed` → `LoopError` (Mode B error against `rpm_for_duty(target_duty)`, gain scaled by `scale_rpm_gain(slope)`) → `Budget::step` → `split_budget` (+ guard overrides + slew clamp) → **`Budget::observe_applied(smoothed cpu_pkg_w + gpu_w)` — the measured draw, never the commanded caps** → CPU write + read-back verdict → GPU PI target at 1 Hz + `verify_lock` verdict; both verdicts share one rule: a candidate `Mismatch` is re-read once and suppressed for 3 ticks after an `on_ac` edge, then `Mismatch` ⇒ `LimitNotSticking` + `Freeze::ActuatorMismatch` + immediate reassert on the same tick, three consecutive ⇒ release to stock with the flag held **while the write + read-back keeps running every reassert period, so a later `Verified` is producible** and clears the strikes; `Unreadable`/`Unverifiable` are non-events, six consecutive `Unreadable` ⇒ `ReadbackBlind` until the next `Verified`. `EcAverage` is seeded from `view.ma_temperature` on auto entry, on re-engagement and at calibration exit, and it plus the fan and steady windows are cleared on a `resumed` sample. `Released` decision ⇒ `release_to_stock`, `Freeze::Released`, flags; a later usable decision re-engages (seed from the floors here; warm-start arrives in fwloop.19). Auto entry requires `lut` only; transitions emit `Noted{mode:...}`; status fields mirrored every tick; the RAPL stickiness watchdog in `on_sample` is retained untouched. **owns:** the auto-mode data flow, the live `EcAverage` instance, gains loading, the budget-bounds derivation (exposed later as `CalibContext.budget_bounds`), the target → `target_duty` snap incl. the no-tread fallback, the RPM window/`fan_valid` supply, the `resync_error` call sites, the verdict rule for both actuators, the `Released` branch, the applied-power report. **consumes:** everything listed in deps. blocked-by fwloop.1: consumes `DutyRpmTable::duty_for_rpm/rpm_for_duty`, `Curve::nearest_tread`. blocked-by fwloop.3: consumes `EcAverage::{push, set_interval, reseed}` and its boxcar contract. blocked-by fwloop.4: consumes `Budget` (`new`, `set_bounds`, `resync_error`, `step`, `observe_applied`, `at_lower_bound_for`), `LoopError`, `Freeze`. blocked-by fwloop.5: consumes `AllocInput{budget_w}`, `split_budget`. blocked-by fwloop.6: consumes `Guards::step`, `effective_target`, overrides. blocked-by fwloop.7: consumes `WriteVerdict`, `verify_lock`. blocked-by fwloop.8: consumes `LoopMode`, `ControlStatus` fields, new flags. blocked-by fwloop.9: consumes `Sample.ec/fanctrl/fanctrl_freshness/fanctrl_view_changed/nvme_temp_c/on_ac`. blocked-by fwloop.10: consumes `Arbiter::decide` → `Decision`. blocked-by fwloop.11: consumes `PersistedState.loop_gains/duty_rpm_table/warm_start`. blocked-by fwloop.22: consumes the tier-free controller. Files: `src/control/controller.rs`. Acceptance: controller unit tests on the fake seams cover: auto entry with LUT only; `Some(gains)` is loaded into `Budget`, `None` uses defaults; the integrator floor tracks a LUT change; an `NVME HOT` tick raises the flag and leaves `target_duty`, T* and the budget **unchanged**; a tick whose snapped duty the curve skips uses `nearest_tread`; a TempLoop tick computes `T* − MA` and moves the budget; an RpmLoop tick on socket Absent drives u from `rpm_for_duty(target_duty) − rpm_smoothed`; a same-name curve edit calls `resync_error` and produces no kick; a fan dropout clears `fan_valid` within one window; **an idle tick whose measured draw sits far below the cap pulls `u` back instead of winding it to the ceiling, and a subsequent load onset does not start from a saturated budget**; a `GPU HOT` override reports the reduced measured draw and u does not grow; a CPU and a GPU `Mismatch` each freeze, flag and reassert on the same tick, and a mismatch within 3 ticks of an `on_ac` edge is suppressed; three consecutive mismatches release to stock with the flag held, the read-back keeps running, and a later `Verified` re-engages without a step; six `Unreadable` raise `ReadbackBlind` with no freeze; a `resumed` sample clears the boxcar and the steady window; a `Released` decision releases the caps, freezes, and a later usable decision re-engages without a step in u; a view change updates the boxcar interval without discontinuity in `ec_ma_c` and a `reseed_ma` decision re-seeds it; `Noted` transitions appear; the RAPL watchdog test still passes. | SPLIT applied (→19) · [cov-1] · [cov-2] split (→22) |
| fwloop.21 | 1 | fwloop.4 | FOPDT fit + IMC gain derivation | `src/calib/fopdt.rs`: `fit_fopdt(&[(t, y)], step_w) -> Option<Fopdt{k,tau,theta}>` by least squares on a step response; `derive_gains(ec: &Fopdt, rpm: &Fopdt, defaults: &LoopGains) -> Option<LoopGains>` with `θ_eff = θ + ma_interval/2` for the EC signal, λ = max(90, 3θ_eff), `Kc = τ/(K(λ+θ_eff))`, `Ti = τ` per signal, `fitted_at` left `None` for the caller to stamp; rejection rules — `K ≤ 0`, `τ < 5 s`, a response magnitude under 3 °C (EC) or 150 RPM (fan), or a derived `Kc` outside [0.25, 4]× the corresponding default — all yielding `None`. The magnitude and ratio bounds are what stop `Kc = τ/(K(λ+θ))` running away as `K → 0⁺`. Pure functions, no I/O. **owns:** `Fopdt`, `fit_fopdt`, `derive_gains`. **consumes:** `LoopGains` (fwloop.4). blocked-by fwloop.4: consumes the `LoopGains` struct. Files: `src/calib/fopdt.rs`, `src/calib/mod.rs`. Acceptance: recovers K/τ/θ within 10 % on a synthetic noisy FOPDT step; derived gains match the IMC formulas (with `θ_eff`) on a known input; each rejection rule fires on its own crafted input — `K ≤ 0`, `τ < 5 s`, a sub-threshold response, and a duty-pinned step whose tiny positive `K_rpm` would otherwise derive a `Kc` orders of magnitude above the default. | [cov-1] split from fwloop.13 · [roast-1] |
| fwloop.13 | 1 | fwloop.4, fwloop.11, fwloop.21 | Calibration step test | `src/calib/runner.rs`: phases `LutSweep → StepTest → Done`; delete `MATRIX_POINTS`, `MatrixPoint`, `Fitting`, `record_matrix_point`, the `fit_batch` use and their tests; `on_sample(&Sample, &CalibContext)` where `CalibContext: Default`; new `src/calib/step.rs`: **burner started first**, then settle detection (EC MA flat ≤ 0.5 °C over 60 s and RPM steady, cap 5 min), then a `budget_bounds.0 + 30 W` step requested via the new `RunnerEffect::SetBudget(w)`, `NeedsLoad` nagged while GPU utilisation is below the pin threshold, 5 min record of EC MA, RPM and **measured** applied power per axis, then `fit_fopdt` + `derive_gains` on the measured per-axis power delta (never the nominal 30 W), stamping `fitted_at` from the sample clock. Two-stage gating: `fanctrl_active` and `!ec_mismatch` before the settle, `argmax_controllable` **after the burner is running** (at the floors the machine's own idle fixture has ambient above apu, so an up-front check self-skips every run). Skips with a `Noted` reason when either gate fails, when the applied power never rose, when the EC max exceeds 95 °C (abort and restore the floors), when the argmax label changes mid-step, or when the fit is rejected. `RunnerEffect::SaveState` now carries `loop_gains`; `progress().phase` reports `"lut"`/`"step"` as plain strings. The controller's one-line call-site change passes `CalibContext::default()` and ignores `SetBudget` until fwloop.19 wires them. **owns:** the `StepTest` phase incl. burner/`NeedsLoad` handling, `CalibContext` struct, `RunnerEffect::SetBudget`, the `fitted_at` stamp, the phase label strings. **consumes:** `LoopGains` (fwloop.4), `PersistedState.loop_gains` (fwloop.11), `fit_fopdt`/`derive_gains` (fwloop.21). blocked-by fwloop.4: consumes `LoopGains`. blocked-by fwloop.11: consumes the `PersistedState.loop_gains` slot. blocked-by fwloop.21: consumes `fit_fopdt` + `derive_gains`. Files: `src/calib/runner.rs`, `src/calib/step.rs`, `src/calib/mod.rs`, `src/control/controller.rs` (call site only). Acceptance: runner end-to-end test on the fake seams walks sweep → burner → settle → step → `SaveState` with gains carrying `fitted_at`, driven by scripted `CalibContext`s; the burner starts before the settle hold and stops after; a scripted run whose idle argmax is uncontrollable but whose loaded argmax is controllable **proceeds** rather than self-skipping; an unloaded step, a 95 °C abort, a mid-step argmax change and a rejected fit each skip with a `Noted` reason and keep defaults; the gain is computed from the measured per-axis delta, not 30 W; no `fit_batch`/`MATRIX_POINTS` symbol remains in `calib/`. | PROMOTE overruled → LEAF (demoted-by-session) · [cov-1] · [cov-2] |
| fwloop.14 | 1 | fwloop.5, fwloop.13, fwloop.15, fwloop.22 | Deletion sweep | Delete `src/control/thermal_model.rs`, `kalman.rs`, `trust.rs`, `cooldown.rs`, `trim.rs`, their `mod.rs` lines, and every remaining reference (the controller's tier and tests go with fwloop.22, the `state.rs` fit tests with fwloop.11, the matrix code with fwloop.13, the telemetry fields with fwloop.15 — this task removes whatever is left, e.g. `gpu_pid.rs` comments, stray imports) plus `TODO.md`/docs mentions. Search the repo for each path and basename in source and non-source files (docs, TODO.md, README, any manifest) — every hit is removed here or owned by fwloop.18 (README). blocked-by fwloop.5: consumes the allocator rewrite that removes the last `contour`/`CONSERVATIVE_START`/`overshoot_settle` references and orphans `trim.rs` (this task deletes the file). blocked-by fwloop.22: consumes the tier-free controller (no importer left in `controller.rs`). blocked-by fwloop.13: consumes the matrix-free runner (no `fit_batch` caller). blocked-by fwloop.15: consumes the telemetry field removal (`trim_rpm`/`gain`/`model_*`). Files: the five modules, `src/control/mod.rs`, `src/control/gpu_pid.rs`, `TODO.md`. Acceptance: a repo-wide search for `thermal_model`, `kalman`, `trust::`, `cooldown`, `trim::`, `adapt_bias`, `ModelSnapshot`, `trim_rpm`, `contour`, `CONSERVATIVE_START`, `overshoot_settle` hits only `docs/research/`, `docs/plans/` history, this spec, and `README.md` (owned by fwloop.18); no `src/` hit remains; `cargo test` and `cargo clippy -D warnings` green. | [cov-1] · [cov-2] re-pointed 12→22 |
| fwloop.15 | 1 | fwloop.8, fwloop.9 | TUI + telemetry surface | `src/ui/view.rs`: header segment `mode A|B|rel · T* · ma · duty → rpm · budget`; render and rank the new flags incl. `READBACK BLIND` (info); calibration progress renders `CalibProgressLite.phase` as the plain string it already is. `src/telemetry.rs`: sample fields `ec_max`, `ec_argmax`, `ec_ma`, `nvme_c`, `fanctrl_speed`, `fanctrl_active`, `strategy`; decision fields `mode`, `t_star`, `budget_w`, `freeze`; remove `trim_rpm`, `gain`, `model_*`. **consumes:** `ControlStatus` fields + flags (fwloop.8); `Sample.ec/nvme_temp_c/fanctrl` (fwloop.9). blocked-by fwloop.8: consumes the `ControlStatus`/`StatusFlag` surface. blocked-by fwloop.9: consumes the new `Sample` fields serialised into telemetry. Files: `src/ui/view.rs`, `src/telemetry.rs`, `src/model.rs`. Acceptance: view snapshot tests for each mode and each new flag; a telemetry line serialises the new fields and omits the removed ones. | [cov-1] · [cov-2] |
| fwloop.16 | 1 | fwloop.1, fwloop.2, fwloop.3, fwloop.9, fwloop.20 | fw-fanctrl emulator + chained plant | `src/test_support/plant.rs` (cfg(test)): `FanctrlEmulator` (1 s tick, boxcar N non-zero with the off-by-one, `eff=min(MA,cur)`, `Curve`, `int()` truncation, `active` switchable, `edit_curve_in_place(points)` under the unchanged strategy name; **two upstream quirks reproduced**: no history append while paused, and a hardcoded 50 °C injected on a scripted sensor-read failure — these are what make the replica's average diverge from the socket's in ways the instantaneous value hides; `view(now) -> FanctrlView` with curve, `ma_temperature`, `ma_interval`, `active`, both stamps, plus a `Freshness` injection hook for socket death), `ThermalPlant` (watts → controllable EC °C, τ 35, θ 20, K 0.8, plus separately labelled `ambient`/`charger` channels and scriptable `gpu_*` channels emitted as an `EcReading`), `FanPlant` (duty → RPM via **its own table**, seeded from the same points with a configurable per-duty offset; one-sided momentum kick on positive slew; ±90 RPM noise from a hand-rolled seeded xorshift, no new dependency) plus **an EC-autofan mode** used whenever the emulator is `active: false`, driving RPM straight from EC temperature with the measured staircase of §Facts (4096 RPM at 61 °C, 4520 at 63, 4658 at 64, flat 4748 across 67–73), scriptable `gpu_temp_c`/`nvme_temp_c` (both `Option`) and `cpu_util`/`gpu_util`/`gpu_sm_mhz` tracks, **a demand model that decides how much of the commanded cap is actually drawn** (so `cpu_pkg_w`/`gpu_w` on the emitted `Sample` are a measured draw that can sit well below the cap — without this the plant cannot exercise the demand-starved wind-up at all), a scriptable `resumed` edge, `ChainedPlant` composing them and producing a full `Sample` per tick (`fanctrl_view_changed` set once per new `print all` view). **owns:** the plant API and its RNG. **consumes:** `Curve` (fwloop.1), `FanctrlView`/`Freshness` (fwloop.2), `EcReading`/`EcLabel` (fwloop.3), the `Sample` field set (fwloop.9), the curve fixtures and the `plant.rs` slot (fwloop.20). blocked-by fwloop.1: consumes `Curve::duty_at`. blocked-by fwloop.2: consumes `FanctrlView` (two stamps) + `Freshness`. blocked-by fwloop.3: consumes `EcReading`/`EcLabel`. blocked-by fwloop.9: consumes the extended `Sample` shape incl. `fanctrl_view_changed`, `fanctrl_freshness`, `ec_valid`, `on_ac`. blocked-by fwloop.20: consumes the `quiet16`/`cool16` fixtures and `src/test_support/plant.rs`. Files: `src/test_support/plant.rs`. Acceptance: emulator reproduces the verified truncation case (T_eff 51.8 → 21 on cool16) and the off-by-one; a scripted temperature drop drives `eff` from the `current` branch; a scripted socket death yields `Absent`; an in-place curve edit yields new points under the same name in the emitted view; `fanctrl_view_changed` is set exactly once per new `print all` view; an open-loop step on the chained plant shows a 26–30 s watts→RPM lag; the plant table offset shifts steady RPM by the configured amount; scripted CPU-heavy vs GPU-heavy utilisation shifts the demand split; a low-demand script emits a measured draw well below the commanded cap; a paused emulator stops appending history, a scripted read failure injects 50 °C, and an `active: false` emulator drives RPM from the measured EC staircase instead of the commanded duty. | [cov-1] · [cov-2] |
| fwloop.17 | 1 | fwloop.12, fwloop.16, fwloop.19 | Closed-loop acceptance + configuration smoke | Controller-level sims on `ChainedPlant` through the real `on_sample`, with fwloop.19's hooks active, the load step being a utilisation + watts step: for each of `quiet16`, `cool16` × TempLoop, RpmLoop (4 runs): load step then 30 min — ≥ 90 % inside ±150 RPM, **no relay under the period-agnostic rule of §5** (no ≥ 3 consecutive sign-alternating band excursions at any period; the count and dominant period are reported); the same 4 runs with plant K/τ/θ perturbed ±50 %; a run with the plant table biased −8 % where refinement brings RPM inside ±150 within 20 min and T* follows the re-snapped duty; **a demand-starved run** (long idle with the plant drawing far below the cap, then a load onset) where `u` never reaches the upper bound and the onset overshoot stays inside ±150; a calibration run (StepTest on the plant, then the quiet16/TempLoop acceptance repeated with the fitted gains meeting the same bar) — **the fit is graded against the boxcar-filtered plant it actually sees, not the raw τ 35 / θ 20 constants**, so the criterion is that the derived `Kc` lands within 25 % of the IMC value for that filtered plant and that the closed loop passes, never that τ/θ match the raw plant; a load-release transient at t=1200 that stays inside ±150 within 90 s; a dGPU-powered-and-hot 30 min run that stays in TempLoop with no `EC MISMATCH`; a dGPU-unpowered run (no `GPU HOT`, floor honoured, `verify_lock` `Unverifiable`); a sub-floor target run that raises `TARGET UNREACHABLE (low)` and holds the floor without relay. Bumpless: socket death at t=600 (A→B), `active:false` at t=700 with a fresh socket (A→B), and an in-place same-name curve edit at t=900 leave |Δu| ≤ one increment and the caps continuous. **`active: false` authority run** (plant in EC-autofan mode, §Facts staircase): with a target below the EC's flat band the loop parks `u` at the floor, raises `TARGET UNREACHABLE (low)` naming the achievable RPM within 60 s, and the integrator does not wind; with the EC on its steep segment below 64 °C the loop does not hunt. `Released`: socket absent **and** invalid fan reading ⇒ stock caps within one hysteresis window, `FANCTRL LOST` + `SENSOR LOST` set, then sensor recovery re-engages RpmLoop from the warm-start without a cap step. Faults: feasibility (T* < ambient + 5), the `high` unreachable case (target above the fans' reach), steep-curve flag, single read-back mismatch freeze, a mismatch suppressed across an `on_ac` edge, three-strike release followed by a `Verified` re-engagement, a 5 min `GPU HOT` episode at the 90 °C threshold with no post-episode overshoot > 150 RPM, an `NVME HOT` episode that raises the flag while the RPM trace is indistinguishable from the same run without it, reconciliation A→B→A with reseed, a scored view skipped because the replica was slewing, and a `resumed` edge mid-run that clears the windows and writes no warm-start or refinement across the gap. Global assertions over every run: only `Speed`/`All` commands were ever recorded by the fake; `ec_ma_c` tracks the emulator's `ma_temperature` within 1 °C in steady state; at least one steady window is detected per converged run. **consumes:** the `on_sample` data flow (fwloop.12), the warm-start/refinement/calibration hooks (fwloop.19), `ChainedPlant` (fwloop.16). blocked-by fwloop.12: consumes the integrated auto loop. blocked-by fwloop.16: consumes `ChainedPlant`. blocked-by fwloop.19: consumes the warm-start/refinement/calibration hooks so the runs grade the final loop. Files: `src/control/sim_tests.rs` (cfg(test)), `src/control/mod.rs`. Acceptance: all listed runs pass deterministically (seeded RNG); each spec-enumerated configuration (2 strategies × 3 modes, dGPU on/off, default vs fitted gains) is exercised end to end (needs: fwloop.12, needs: fwloop.19). | [cov-1] · [cov-2] |
| fwloop.18 | 1 | fwloop.2, fwloop.6, fwloop.8, fwloop.13 | README + docs | README: rewrite Calibration walkthrough (LUT sweep + step test, burner, `NeedsLoad`), Auto mode (cascade, modes, flags table), Safety model (fw-fanctrl owns the fans, socket read-only by construction, guards, read-back incl. `READBACK BLIND`), Configuration table (drop `online_rls`, add `fanctrl_socket`, `gpu_hot_c`, `nvme_hot_c`), and a Safety-model line saying plainly that the NVMe reading is reported and not acted on, with the measurement behind that (§2.8); `docs/research/03-control.md` pointer note; INDEX status → implemented. **consumes:** config key names (fwloop.2, fwloop.6), the `StatusFlag`/`LoopMode` set (fwloop.8), calibration flow (fwloop.13). blocked-by fwloop.2: consumes the `fanctrl_socket` key name/default. blocked-by fwloop.6: consumes the guard key names/defaults. blocked-by fwloop.8: consumes the flag and mode names for the flags table. blocked-by fwloop.13: consumes the step-test flow (durations, skip reasons). Files: `README.md`, `docs/research/03-control.md`, `docs/superpowers/specs/INDEX.md`. Acceptance: every config key in `config.rs` appears in the README table and vice versa; every `StatusFlag` variant appears in the flags table; no README mention of matrix/model/trim/RLS remains. | [cov-1] |
| fwloop.19 | 1 | fwloop.12, fwloop.13 | Controller hooks: warm-start, refinement, calibration | `src/control/controller.rs`: the steady-window detector on the **`rpm_smoothed`** series (population stdev < 60 over 40 s, `active`, the view's own `speed_pct` equal to `target_duty` for the whole window, no guard override, `u` off both bounds — §2.3, so a window whose achieved duty differs from the target's never writes into the target's entry) that (a) records `warm_start[key(strategy, duty, on_ac)] = u` and (b) calls `DutyRpmTable::refine(duty, mean_rpm)`, and when the snapped duty changes surfaces the new `target_duty` to the arbiter (T* derivation stays in fwloop.10; the controller calls `resync_error` on `t_star_changed`); warm-start seeding of `u` **only** on auto entry, on re-engagement from `Released`, and at calibration exit (fallback: the floors) — a mid-session key change re-keys the recording target and never re-seeds (§2.4); building `CalibContext` each sample from the arbiter's decision and the budget bounds; asserting `Freeze::Calibrating` for the whole calibration session (LUT sweep included) and applying `RunnerEffect::SetBudget` (seed `u = w`, normal split + command path); persisting the table, warm-start map and `loop_gains` through `save_persisted_state`. **owns:** the steady-window detector, warm-start seed/record and the no-reseed-on-key-change rule, refinement trigger, the whole-session calibration freeze, the `CalibContext`/`SetBudget` ends on the controller side. **consumes:** the wired auto loop incl. budget bounds and `resync_error` (fwloop.12), `CalibContext`/`SetBudget` (fwloop.13), `WarmStart` (fwloop.4, transitively through 12), `DutyRpmTable::refine` (fwloop.1, transitively). blocked-by fwloop.12: consumes the integrated `on_auto_sample`, `EcAverage` instance and budget bounds. blocked-by fwloop.13: consumes `CalibContext` and `RunnerEffect::SetBudget`. Files: `src/control/controller.rs`. Acceptance: a steady window on the smoothed series records both the warm-start entry and a table refinement, and a window on raw ±90 RPM noise still qualifies; auto entry seeds `u` from a matching key, floors otherwise; a strategy change, a snapped-duty change and an AC unplug each re-key without re-seeding (that tick's Δu equals the ordinary PI increment); re-engagement from `Released` seeds from the warm-start; the integrator is frozen from calibration start through exit and `u` is unchanged across a LUT sweep; a `SetBudget` effect lands the requested budget through `split_budget` and commands it; `CalibContext` mirrors the arbiter's decision and the budget bounds; the persisted file round-trips all three. | [cov-1] · [cov-2] |
| fwloop.23 | 1 | fwloop.1, fwloop.2, fwloop.3, fwloop.4, fwloop.5, fwloop.6, fwloop.7, fwloop.8, fwloop.9, fwloop.10, fwloop.11, fwloop.12, fwloop.13, fwloop.14, fwloop.15, fwloop.16, fwloop.17, fwloop.18, fwloop.19, fwloop.20, fwloop.21, fwloop.22 | Integration sweep: fw-fanctrl closed loop | Verify the goal's main flows end to end on the merged tree and implement what is missing: (1) engage auto from a fresh `state.json` with only a LUT, walk TempLoop → socket death → RpmLoop → recovery → TempLoop, run a calibration, restart the daemon and confirm the warm-start, table and gains reload; (2) sweep for unwired config values, parameters and interfaces — every `Config` key is read somewhere, every `StatusFlag` is raised somewhere and rendered, every telemetry field is populated, every `Effect` variant is applied, `CalibContext` fields all originate from live data; (3) add the integration tests no per-task test covers (sampler → controller → telemetry line with real types; config → poller construction; a full `on_command`/`on_sample` session on the fakes); fix small gaps inline, file blockers for large ones. blocked-by fwloop.1: consumes all leaves (integration sweep). blocked-by fwloop.2: consumes all leaves (integration sweep). blocked-by fwloop.3: consumes all leaves (integration sweep). blocked-by fwloop.4: consumes all leaves (integration sweep). blocked-by fwloop.5: consumes all leaves (integration sweep). blocked-by fwloop.6: consumes all leaves (integration sweep). blocked-by fwloop.7: consumes all leaves (integration sweep). blocked-by fwloop.8: consumes all leaves (integration sweep). blocked-by fwloop.9: consumes all leaves (integration sweep). blocked-by fwloop.10: consumes all leaves (integration sweep). blocked-by fwloop.11: consumes all leaves (integration sweep). blocked-by fwloop.12: consumes all leaves (integration sweep). blocked-by fwloop.13: consumes all leaves (integration sweep). blocked-by fwloop.14: consumes all leaves (integration sweep). blocked-by fwloop.15: consumes all leaves (integration sweep). blocked-by fwloop.16: consumes all leaves (integration sweep). blocked-by fwloop.17: consumes all leaves (integration sweep). blocked-by fwloop.18: consumes all leaves (integration sweep). blocked-by fwloop.19: consumes all leaves (integration sweep). blocked-by fwloop.20: consumes all leaves (integration sweep). blocked-by fwloop.21: consumes all leaves (integration sweep). blocked-by fwloop.22: consumes all leaves (integration sweep). Files: `src/**` (small inline fixes), `src/integration_tests.rs` (cfg(test)). Acceptance: the three flows above pass on the fakes; the unwired-sweep checklist is recorded in the spec's Post-Implementation Notes with zero open items or a filed blocker per item; `cargo test` and `cargo clippy -D warnings` green. | root integration sweep |

Graph shape: longest chain fwloop.20 → 2/3 → 9 → 12 → 19 → 17 → 23 (7 rounds). Round 1
readies fwloop.20, 1, 4, 5, 6, 8; fwloop.22 (tier removal) now lands in round 2 abreast of the
sensor work, so fwloop.14 can run as soon as 13 and 15 have landed instead of waiting for the
hub. fwloop.12 is pure wiring; fwloop.19 carries the separable hooks; fwloop.17 waits for 19 so
the acceptance grades the shipping loop; fwloop.23 is the terminal join.

## Post-Implementation Notes

*As this design is implemented and iterated on — bug fixes, adjustments, anything that diverged from the assumptions above — append a dated note here, whether or not a formal debugging skill was used.*

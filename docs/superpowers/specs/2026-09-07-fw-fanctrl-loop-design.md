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
- `FanctrlView { strategy, active, speed_pct, temperature, ma_temperature, ma_interval, curve:
  Vec<(temp, speed)>, update_freq, observed_at }`. Staleness: no successful `print all` for 90 s or
  no successful `print speed` for 15 s ⇒ `stale`. Connection refused / ENOENT ⇒ `absent`.
- `Curve`: piecewise-linear in **file order** (never sorted), flat clamp below the first and above
  the last point, `int()` truncation — a faithful copy of `FanController.py`. Provides:
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
- `EcLabel` classifies by label prefix: **controllable** = `apu`, `cpu`; **uncontrollable** =
  `ambient`, `charger`; **gpu_*** = ignored for controllability (they are 0/ENODATA unless the
  dGPU is powered, and the dGPU has its own loop).
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
- Passive refinement: when the controller observes a steady window (RPM population stdev < 60
  over ≥ 40 s **and** commanded duty constant over the window **and** `active: true`), the entry
  for that duty is updated `rpm ← 0.8·rpm + 0.2·mean` (created if absent). Persisted in
  `state.json`. A refinement that changes `duty_for_rpm(target)` re-derives T* (bumpless: the
  budget is untouched).
- If `tread(d)` is `None` for the snapped duty (the curve skips that integer), snap to the nearest
  duty with a tread, preferring lower.

### 2.4 `control/budget.rs` — the single integrator (new)

- Velocity-form PI, period `PI_PERIOD_S = 5` (the allocator cadence), no derivative term.
  `Δu = Kc·(e_k − e_{k−1}) + (Kc·Ts/Ti)·e_k`; `u` is the total budget in watts.
- `u ∈ [cpu_floor_w + gpu_floor_w(lut), cpu_max_w + gpu_max_w]` where `gpu_floor_w` is the LUT's
  watts at `gpu_floor_mhz`. Clamping anti-windup plus back-calculation with `Tt = Ti`.
- Error source is a `LoopError` enum supplied by the arbiter each tick:
  `Temp { e_c: f64 }` (T* − MA, gains `kc_w_per_c`, `ti_s`), `Rpm { e_rpm: f64 }` (target −
  smoothed RPM, gains `kc_w_per_rpm`, `ti_rpm_s`). **Both integrate into the same `u`**, so a mode
  switch changes only the next increment — bumpless by construction. On a switch, `e_{k−1}` is
  reset to the new mode's current error (no proportional kick).
- `Freeze` reasons (no integration, `u` held): `ActuatorMismatch`, `FanctrlStale`,
  `FanctrlPaused` (`active: false` in Mode A), `ArgmaxUncontrollable`, `Calibrating`,
  `Released`. Freeze is reported in the decision telemetry.
- `Gains`: persisted `LoopGains { kc_w_per_c, ti_s, kc_w_per_rpm, ti_rpm_s, tau_s, theta_s,
  k_c_per_w, k_rpm_per_w, fitted_at }` or the defaults `kc_w_per_c = 0.4`, `ti_s = 35`,
  `kc_w_per_rpm = 0.4 / 55` (one duty point ≈ 55 RPM ≈ one tread ≈ 1 °C on quiet16),
  `ti_rpm_s = 35`.
- Warm-start: on auto entry and on every (strategy, target duty, AC) key change the integrator
  seeds `u` from `warm_start[key]` if present, else from the floors. The **current** `u` is written
  to `warm_start[key]` whenever the loop has been steady (RPM stdev < 60 over 40 s) — that is
  what "last settled budget" means.

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
- Every transition emits `Effect::Noted { cause: "mode:<from>-><to>:<reason>" }` and a
  `StatusFlag`/telemetry `mode` field. Entering TempLoop re-derives T* and resets `e_{k−1}` —
  the budget `u` is never touched by a transition.
- Hysteresis: a mode must hold its conditions for 3 consecutive ticks (15 s) before the arbiter
  switches **into** it; switching **out** of TempLoop on a hard fault (`absent`, `!active`,
  `!ec_valid`) is immediate.

### 2.6 Reconciliation (`EC MISMATCH`)

On every successful `print all`, compare the replica's `max_c` for the sample nearest
`observed_at` with the socket's `temperature`. Three consecutive polls with |Δ| > 1 °C set
`EC MISMATCH` (TempLoop unavailable → RpmLoop); three consecutive polls with |Δ| ≤ 1 °C clear it
and re-seed `EcAverage` from `ma_temperature`. The comparison also runs once before the first
TempLoop entry after auto is engaged.

### 2.7 Feasibility and steepness

- **Feasible** iff `T* ≥ max(uncontrollable readings) + 5 °C`. Infeasible ⇒ `TARGET
  UNREACHABLE` flag (existing), mode falls to RpmLoop whose integrator will clamp at the floors;
  the flag text explains why (`T*=61 < ambient 58+5`). The flag clears when feasible again for 60 s.
- **Steep** iff `slope_at(T*) > 2 %/°C` ⇒ `STEEP CURVE` flag (warning only; the loop runs). On
  cool16 every tread above 70 °C is steep; on quiet16 none is below 88 °C.

### 2.8 Guards (`control/guards.rs`, new)

Both run every sample in any auto mode, ahead of the arbiter, with hysteresis
(`enter` threshold, `exit = enter − 5 °C`).

- **dGPU** (`gpu_hot_c`, default 83; NVML temperature already sampled): while hot, the GPU share
  from `split_budget` is overridden to `max(gpu_floor_w, current gpu_w − DOWN_RATE_W)` each 5 s
  allocator tick;
  flag `GPU HOT`. The integrator is not frozen — the budget the CPU can't use simply goes unused.
- **NVMe** (`nvme_hot_c`, default 80; new hwmon read of the `nvme` chip's `Composite`): while
  hot, the effective RPM target is `min(target + nvme_boost_rpm, FAN_TARGET_MAX_RPM)` (default
  boost 500), re-snapped through the table; flag `NVME HOT`. The user's configured target is
  unchanged.

### 2.9 Actuator read-back

- **CPU.** `set_sustained_mw` writes `--slow-limit=<mw> --stapm-limit=<mw> --fast-limit=<fast>`
  then runs `ryzenadj --info` and parses the table rows `PPT LIMIT SLOW`, `PPT LIMIT FAST`,
  `STAPM LIMIT` (values in W with 3 decimals). Verified iff slow and fast are within 0.5 W of the
  commanded values; STAPM is written but **not** required to verify (reported to fail silently on
  this SoC). Returns `Verified(w)` / `Mismatch { field, commanded, read }` / `Unreadable`.
- **GPU.** No NVML read of the lock exists; verification is the measured SM clock: while
  `gpu_util > 90 %`, `gpu_sm_mhz ≤ locked + 30` for 3 samples (the LUT sweep's pin rule reused).
  Below 90 % utilisation the write is `Unverifiable` (not a failure).
- `Mismatch` ⇒ `LIMIT-SLIP!` (existing flag), integrator `Freeze(ActuatorMismatch)`, immediate
  reassert; three consecutive mismatches ⇒ release to stock with the flag held (the existing
  stickiness watchdog behaviour). The RAPL stickiness watchdog stays as a second, independent check.

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
`ModelDistrust`, adds `FanctrlLost`, `EcMismatch`, `SteepCurve`, `GpuHot`, `NvmeHot`.
`Effect::ModelSnapshot` is removed. Decision telemetry (`AutoAllocated`) carries `mode`,
`error`, `budget_w`, `freeze`.

### 3.3 Calibration (`calib/runner.rs`)

Phases: `LutSweep` → `StepTest` → `Done`. `MATRIX_POINTS`, `MatrixPoint`, `Fitting`,
`record_matrix_point` and the `fit_batch` path are deleted.

`StepTest` (needs fw-fanctrl `active`, replica reconciled, argmax controllable; otherwise the
phase is skipped with a `Noted` reason and defaults stay in force):

1. Hold `u` at the floors until the EC moving average is flat (≤ 0.5 °C change over 60 s) and
   RPM steady; cap 5 min.
2. Step `u` by `STEP_W = 30` W split by demand (burner on for CPU as today; GPU load is
   user-provided as today — nag `NeedsLoad` if GPU utilisation stays low), hold 5 min or until
   the EC average has been flat for 90 s.
3. Fit a first-order-plus-dead-time model by least squares on the step response, separately for
   the EC average (`k_c_per_w`, `tau_s`, `theta_s`) and for RPM (`k_rpm_per_w`, `tau_rpm`,
   `theta_rpm`). Derive `λ = max(90, 3·θ)`, `Kc = τ / (K·(λ + θ))`, `Ti = τ` for each.
4. Persist `LoopGains` with `fitted_at`. A fit with `K ≤ 0` or `τ < 5 s` is rejected (defaults
   kept, `Noted` reason).

Fans are never commanded during calibration; the step is applied through the normal caps with
the integrator frozen (`Calibrating`).

### 3.4 Sensors, sampler, config

- `sensors/ec.rs` (2.2) and an `nvme_temp_c` reader added to the sampler; `Sample` gains
  `ec: Option<EcReading>`, `ec_valid`, `nvme_temp_c: Option<f64>`, `fanctrl: Option<FanctrlView>`,
  `on_ac: bool` (from `/sys/class/power_supply/ACAD/online`).
- Config keys added: `fanctrl_socket` (path), `gpu_hot_c`, `nvme_hot_c`, `nvme_boost_rpm`.
  Removed keys are tolerated like `online_rls` today.

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
  ±150 RPM, no sustained relay cycle (no ≥ 3 consecutive band excursions with period 60–200 s).
  Run once per active strategy (`quiet16`, `cool16`) and once in each mode.
- Bumpless tests: socket death mid-run (A→B) and strategy edit mid-run leave `u` continuous
  (|Δu| ≤ one PI step) and the caps continuous.
- Feasibility (`T*` below ambient + 5), steep curve, read-back mismatch (freeze + flag), `GPU HOT`
  (GPU share decays, budget unchanged), `NVME HOT` (target boost, re-snap), reconciliation
  (three mismatches → B, three matches → A with re-seeded average).
- The existing `Runner`/`FakeRunner` and `GpuClockCtl`/`FakeGpu` seams stay the only test seams;
  the socket client gets a `FanctrlSource` trait with a fake.

## 6. Follow-ons (not in this tree)

- Gain scheduling of `Kc` on commanded duty (power-law 0.8, clamp [0.5, 2]×).
- A slow outer RPM trim on T* (λ 300–600 s, ±1 tread authority) if passive table refinement
  proves insufficient in the field.
- RyzenAdj `--tctl-temp` as a hardware CPU leg.
- Upstream note to fw-fanctrl about the 0777 socket + `set_config`-as-root.

## Post-Implementation Notes

*As this design is implemented and iterated on — bug fixes, adjustments, anything that diverged from the assumptions above — append a dated note here, whether or not a formal debugging skill was used.*

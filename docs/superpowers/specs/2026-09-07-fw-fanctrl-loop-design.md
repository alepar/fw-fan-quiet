# Closing the loop on fw-fanctrl (no learned thermal model)

**Date:** 2026-09-07
**Status:** designed — 2 coverage rounds + 3 super-roast iterations applied
**Tracker:** beads epic `fw-fanctrl-loop-6ma` (24 tasks, imported from the table below;
the table stays the human-readable source of truth for task content)
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
  **Limitation, stated because the numbers are used as a plant:** the run ramped up under load
  and then decayed, so the medians mix the rising and falling branches. The flat 67–73 °C
  plateau is well sampled (n = 102 at 71 °C) and is the part the authority argument rests on.
  The steep 61–64 °C segment is thin (n = 4 at 64 °C) and its rising-branch shape and any
  hysteresis width are **not** established. `fwloop.16` therefore treats the plateau as measured
  and the steep segment as approximate, and `fwloop.17`'s steep-segment hunt run is a check on
  the loop, not evidence about the EC. A slow 55 → 75 °C ramp with a separate decay, logged at
  1 Hz, would settle it and is listed in §6.
- `ryzenadj --info` fails while `ryzen_smu` is loaded — the same precondition the existing
  `smu_module.rs` already enforces for writes, so read-back shares it.
- `nvidia-smi` reports `power.limit` N/A and `enforced.power.limit` 100 W; we control locked
  clocks, not power, so the GPU read-back is the measured SM clock under load.
- **Measured 2026-09-09, `DEMAND_MARGIN_W` per axis (§2.4's anti-windup spike, `fwloop.24`,
  bead `fw-fanctrl-loop-9it`).** Method: command a sustained cap, drive a load, sample
  commanded-vs-drawn power at the 5 s allocator cadence for 7 ticks after a settle window, at
  several cap levels and load compositions.
  - **CPU: unloaded `ryzen_smu` and left it unloaded for the duration** (the precondition above
    — `ryzenadj --info` needs it unloaded; unloading and immediately reloading, as a prior
    attempt did, restores the broken precondition before `ryzenadj` runs). With `ryzen_smu`
    unloaded, `sudo ryzenadj --stapm-limit=<mw> --slow-limit=<mw> --fast-limit=53000` plus
    `stress-ng --cpu 24` (24-thread AVX matrixprod, full saturation) at 20 W and 35 W caps:
    `PPT VALUE SLOW` (drawn) tracked `PPT LIMIT SLOW` (commanded) within 0.005–0.08 W across all
    14 samples, RAPL `energy_uj` (root, `/sys/class/powercap/intel-rapl:0`) cross-validated to
    within 0.01 W of the same commanded value over the sampling window. The same ≤ 0.08 W noise
    floor held under lighter, genuinely cap-bound loads too (2-thread `stress-ng`, a single
    15%-duty-cycled thread, and ordinary desktop background load, all at the 35 W cap — this
    machine's live desktop background draw turned out to already sit above 35 W, so those
    "light" loads were still cap-bound, not demand-limited, on this run). A genuine
    demand-limited gap was measured directly instead: at a 54 W cap (near-unconstrained) with
    only ordinary desktop background load, drawn power settled to 36.8 W (RAPL) — a 17.2 W gap.
    **`DEMAND_MARGIN_W_CPU = 2.0 W`** — ~25x the measured noise floor, ~8x below the smallest
    measured genuine gap. Module state was restored (`modprobe ryzen_smu`) and stock CPU limits
    reasserted (platform-profile toggle) after the measurement; no stray load processes were
    left running.
  - **GPU:** `nvidia-smi -lgc 210,1500` (applied clock read back 1492 MHz) with a `glxgears`
    fill/geometry-bound proxy load at 95–100% reported utilization: `power.draw` settled 13.50 →
    13.39 W over 7 ticks, a 0.11 W spread. **Limitation, stated because this number is used as a
    margin:** `glxgears` is fill/geometry-bound, not compute-bound (unlocked, it draws only
    24.7 W of the card's 100 W limit at 97–100% reported utilization), so this spread likely
    understates the noise floor a real compute-bound game workload would show; a heavier
    generator (`glmark2`/`vkmark`/CUDA) was not available on this machine (same
    "Limitation, stated because the numbers are used as a plant" pattern as the EC autofan curve
    fact above). A genuine demand gap was measured directly at the same lock: idle (no
    artificial load) settled to 7.27 W vs the proxy-loaded 13.4 W above — a 6.1 W gap.
    **`DEMAND_MARGIN_W_GPU = 3.0 W`** — ~27x the measured noise floor (proportionally larger
    than the CPU margin's ratio, since the GPU noise-floor number itself is the weaker
    measurement) and about half the smallest measured genuine gap. The clock lock was released
    (`nvidia-smi -rgc`) after the measurement.

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
  A rejected curve means TempLoop is unavailable **indefinitely**, so it gets its own flag,
  `CURVE INVALID`, at warning severity, and its own mode reason. It must not reuse
  `STEEP CURVE`, whose own definition says the loop still runs and whose severity is
  informational — a permanent loss of Mode A surfacing as the least alarming flag in the set,
  under a label that is false in this case, is how it would go unnoticed. The arbiter takes
  curve validity as an input and falls to RpmLoop.
  - `duty_at(t: f64) -> u8`
  - `tread(d: u8) -> Option<(t_lo, t_hi)>`: the maximal temperature interval where
    `duty_at(t) == d`, computed by **the same rule at every duty** — `min_duty`/`max_duty` (the
    curve's own floor/ceiling) get no special case. `None` when no temperature yields exactly
    `d` (the target then snaps to the nearest reachable duty, see 2.3).
    **Endpoint semantics (settled here; was a deferred minor on Task 1, `fw-fanctrl-loop-9dv`,
    never signed off before Task 16, `fw-fanctrl-loop-iym`, merged as its first consumer —
    `fw-fanctrl-loop-nez` closes the gap):** the interval is unbounded in principle at the
    curve's own floor and ceiling — fw-fanctrl would report that duty at any arbitrarily
    low/high temperature past the curve's own defined domain — but `tread` is **the unbounded
    interval intersected with the curve's own domain**, `[points.first().0, points.last().0]`,
    never left open with a `NEG_INFINITY`/`INFINITY` endpoint. Concretely:
    - **Where the curve defines a genuine flat run at that extreme** (both `quiet16` and
      `cool16` have one at their floor: 0→55 °C at duty 15, 0→50 °C at duty 20), the
      intersection is a real, finite, non-empty interval — same shape as any interior duty's
      tread, just clamped at the domain edge instead of a neighbouring point. T* lands inside
      it and Mode A runs there exactly as it would anywhere else: **the quietest reachable
      target keeps the temperature loop**, it does not permanently fall to RpmLoop.
    - **Where the extreme duty is attained only instantaneously, at a single defining point with
      no flat run** (both curves' *ceiling*: quiet16 and cool16 each reach their top duty only
      at their very last point), the intersection is empty and `tread` returns `None` for that
      exact duty. This is not by itself `TARGET UNREACHABLE`: 2.3's `nearest_tread` already
      snaps a duty with no tread of its own to the nearest one that has one, and the immediately
      adjacent duty always does (one integer step is a small enough temperature step that the
      curve attains it over a real, if narrow, interval) — so a target sitting exactly on such a
      ceiling still resolves to a finite T* one duty in (quiet16's 100 resolves via 99, T* ≈
      94.9 °C), and Mode A keeps running there too, not just at the floor.
    - The two ends are **not required to behave identically, and for both of this design's own
      curves do not** — a flat lead-in and an instantaneous ceiling are different curve shapes,
      not an arbitrary per-end special case. It is one rule (intersect with the domain) applied
      uniformly; the asymmetric *outcome* is a fact about `quiet16`/`cool16`, not about `tread`.
      A hypothetical curve with a flat run at its ceiling too (e.g. an extra point repeating the
      max duty) would get a finite ceiling tread from the same rule, no code change required.
    - `slope_at(t_star)` stays meaningful in both cases above: whenever `t_star` is `Some`, it
      lies inside `[points.first().0, points.last().0]` by construction (never on the flat
      clamp, where `slope_at` is defined as `0`), so `STEEP CURVE` is judged on the same real
      segment slope as any interior duty — never coerced by an out-of-domain 0.
    - The §2.7 "unreachable from above" bound-hold rule is unaffected either way: it reads
      `Budget::at_upper_bound_for()` and the tick's `error_sign` directly, never `t_star`, so it
      fires exactly as before regardless of which duty's tread ends up backing T* at the
      ceiling.
    - Net effect on the curve/arbiter seam (`fw-fanctrl-loop-nez`): `tread`/`t_star` can no
      longer return a non-finite value for **any** duty in `[min_duty, max_duty]`, on either
      curve, at any point — the velocity-form PI (2.4) never again sees an infinite error.
  - `t_star(d) = (t_lo + t_hi) / 2` — always finite when `tread(d)` is `Some`, by the above;
    `None` exactly when `tread(d)` is `None`.
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
  `ma_interval` samples (or a seed) before its mean is used as one. `unreconciled` is a distinct
  reason string on the mode decision and appears in the telemetry, so a controller sitting in
  RpmLoop because no view has been scored yet is distinguishable from one held there by a real
  mismatch. At one scored view per 30 s poll, expect the first comparison within ~30 s of
  engaging auto and `EC MISMATCH` to need ~90 s to latch.
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
  to the current error without touching `u`; the controller calls it on a mode switch, **on
  every T* or snapped-target re-derivation** (strategy edit, curve edit, table refinement), and
  on leaving any freeze, so the velocity form never converts a setpoint jump into a
  proportional kick.
- `Budget::new(&LoopGains)` / `set_gains(&LoopGains)`: the controller loads
  `PersistedState.loop_gains` on auto entry, `LoopGains::default()` when absent.
- `Freeze` reasons (no integration, `u` held): `ActuatorMismatch`, `Calibrating`, `Released`,
  `DemandLimited` (below).
  Freeze is reported in the decision telemetry. (Socket staleness, `active: false` and an
  uncontrollable argmax are not freezes — the arbiter moves the loop to RpmLoop instead, §2.5.)
- `Gains`: persisted `LoopGains { kc_w_per_c, ti_s, kc_w_per_rpm, ti_rpm_s, tau_s, theta_s,
  k_c_per_w, k_rpm_per_w, fitted_at }` or the defaults below. The regulated variable is the
  boxcar mean, not the raw EC reading, so the effective dead time is `θ + N/2 ≈ 50 s`, not 20 s;
  the defaults are derived with that `θ_eff` (λ = max(90, 3·θ_eff) = 150), giving
  `kc_w_per_c = τ/(K·(λ+θ_eff)) = 35/(0.8·200) = 0.22` W/°C, `ti_s = τ = 35`. The λ = 90 floor is
  **not** the boxcar's margin — it is a separate lower bound. **The `θ_eff` substitution belongs
  to the raw-domain plant only**, i.e. to these defaults; a fit performed on the already-filtered
  EC average in §3.3 measures a `θ̂` that *includes* the filter, so adding `ma_interval/2` again
  would count the boxcar twice and roughly halve the fitted `Kc`.
- Mode B's defaults come from the same derivation on the RPM leg, at the reference slope: one
  duty point is ≈ 78 RPM on the seeded table (30 → 2560, 36 → 3030), and at `slope_ref` = 1 %/°C
  that is 78 RPM/°C, so `K_rpm = 78 · 0.8 = 62.4` RPM/W. With the same τ 35, θ_eff 50, λ 150:
  **`kc_w_per_rpm = 35/(62.4·200) = 0.0028` W/RPM, `ti_rpm_s = 35`.** These are the numbers the
  slope schedule below multiplies; without them stated, `scale_rpm_gain` has nothing to scale.
- **Mode B's gain is scheduled on the live curve, not fixed.** `kc_w_per_rpm` assumes a
  RPM-per-watt plant gain that scales with the curve's slope at T*: a flat quiet16 tread and a
  steep cool16 tread differ by roughly a factor of four. The controller therefore scales the
  persisted or default `kc_w_per_rpm` by `slope_ref / max(slope_at(T*), slope_ref)` with
  `slope_ref = 1.0 %/°C`, clamped to [0.25, 1]×, so the steepest supported tread gets the
  smallest gain. This is the minimal form of the gain scheduling §6 defers; without it the
  cool16 × RpmLoop acceptance run is required to pass on a plant four times more sensitive than
  the one the default was derived for. **RpmLoop is reachable with no curve at all** — socket
  absent from boot, a rejected non-monotone curve, a `Released` recovery with the socket still
  dead — so `Decision.slope` is an `Option`: `None` means no resolved curve and the schedule
  applies the conservative `0.25×` clamp rather than the `1×` that a defaulted zero slope would
  silently produce.
- Warm-start: **only on auto entry** (and on re-engaging from `Released` or at calibration
  exit) the integrator seeds `u` from `warm_start[key]` if present, else from the floors. A
  mid-session key change (strategy edit, snapped-duty change, AC↔battery) never re-seeds — it only
  changes which key the next steady window records into. This keeps §2.5's rule that `u` is never
  touched by a transition. The **current** `u` is written to `warm_start[key]` whenever the loop
  has been steady (RPM stdev < 60 over 40 s) — that is what "last settled budget" means.
- **Demand-limited anti-windup — invariant fixed here, exact rule settled by a spike
  (fwloop.24).** The integrator must not wind up while nothing is consuming the budget: at idle
  or in a game lull the temperature error stays positive, and an unchecked integrator saturates
  at `cpu_max_w + gpu_max_w` so the next load onset runs uncapped through the whole dead time.
  The old design's conservative start and model contour covered this; they are deleted.

  **This spec did not originally state the replacement rule, on purpose.** Three prose revisions
  of it were each confirmed Blocking by an independent adversarial panel, and each failure was
  introduced by the previous fix — a back-calculation toward the draw that turned out to be a
  tracker, then a freeze that turned out to latch. That was a signal the rule could not be
  settled by writing more prose about it. `fwloop.24` (bead `fw-fanctrl-loop-9it`) built a
  throwaway harness (the real `Budget`, a first-order thermal plant driven by **actual drawn
  watts** rather than commanded `u`, a demand model, and the real `allocator::split_budget`) and
  **measured** four candidate rules against every scenario the three review rounds named. What
  follows is the result: the rule this section now states normatively, and — below the fixed
  invariants — the measurements and the sweep that justify it.

  What **is** fixed, and what the decided rule below must not violate (unchanged by the spike):
  - **Anti-windup is directional.** Accumulation may be halted only in the direction that
    deepens the condition — the standard conditional-integration form. A rule that also blocks
    integration in the *recovering* direction can self-latch, which is exactly how revision three
    failed: it froze the descent that was the loop's only route back to a binding cap, and the
    fans sat above target for the session.
  - **Never pull `u` toward the draw.** An ungated back-calculation toward measured power is a
    tracker, not anti-windup: its equilibrium is `u = draw + Kc·e`, so every lull drags the cap
    onto the lull's consumption. Back-calculation belongs against the **bounds** only, which the
    clamping rule above already does and is the only back-calculation this design performs.
  - **Judge each axis separately.** A combined-sum test makes any structurally undrawn component
    (the GPU floor share while the dGPU is unpowered) a permanent gap, and a sum-based rule would
    close it by lowering `u` until the CPU sits pinned at its floor.
  - **Whatever holds `u` is visible.** An active hold appears in the status line and the decision
    telemetry, so a loop that is deliberately not integrating is distinguishable from one that is
    converged.

  #### The decided rule (settled 2026-09-09 by `fwloop.24`, `src/control/spike_antiwindup.rs`)

  **Per axis, `ConditionalHysteresis`.** Every allocator tick, for each axis `i` independently:

  ```text
  demand_limited(i) := cap_i > floor_i + ε   AND   (cap_i − draw_i) > DEMAND_MARGIN_W(i)
  ```

  `cap_i` is the axis's **post-guard-override** commanded cap for this tick — the value
  `split_budget` actually produced after `gpu_share_override` (§2.8) has been applied to
  `gpu_max_w`, not a hypothetical pre-override value. There is only one cap an axis is ever
  actually offered in a given tick; comparing draw against anything else would compare it against
  power that was never really available. `floor_i` is that axis's own configured floor
  (`cpu_floor_w`, or the LUT's watts at `gpu_floor_mhz`) — **not** zero and **not** a
  powered/unpowered flag. The `cap_i > floor_i + ε` guard is the load-bearing half of "judge each
  axis separately": it is what excludes an axis that was never offered headroom above its floor
  (a structurally-undrawn GPU sits with `cap == floor` every tick, since `split_budget`'s
  demand-proportional split gives a zero-demand axis nothing beyond its floor) from ever
  registering as "wasting unused headroom" — see the sweep's dedicated predicate tests below for
  why a naive per-axis check *without* this guard, or a sum-of-axes check, both fail this exact
  case.

  The **halt** applied to the integrator each tick is `any(demand_limited(i) for i in {cpu, gpu})
  AND error_sign > 0` (`error_sign` = `sign(e_c)`, the *current* tick's Mode A/B error before this
  tick's step) — one axis's condition is enough to gate the shared scalar `u`, but only in the
  deepening direction; `Budget::step`'s existing `Freeze::DemandLimited` handling already applies
  that gate correctly (see `src/control/budget.rs`, unchanged by this task).

  **Hysteresis: `HYSTERESIS_DWELL_TICKS = 2`** (10 s at the 5 s allocator cadence), applied
  per-axis, symmetric on entering and leaving. A candidate's raw per-tick verdict must persist for
  2 consecutive ticks before the effective (debounced) state changes. 10 s is short relative to
  `Ti = 35` s, so it costs negligible recovery latency, but the sweep's scenario 7 (draw held
  right at the margin, ±noise) shows it cuts hysteresis-state chatter from 81 raw transitions to
  29 over a 100-tick window — worth having; `Conditional` (no hysteresis) is not the chosen rule.

  **`DEMAND_MARGIN_W` (§Facts has the full measurement):**
  - CPU: **2.0 W**. `ryzenadj --info`'s `PPT VALUE SLOW` (drawn) against `PPT LIMIT SLOW`
    (commanded) tracked within ≤ 0.08 W across four cap-bound load compositions, RAPL
    `energy_uj`-cross-validated to ≤ 0.01 W at the two heaviest; a genuine demand-limited gap,
    measured directly, was 17.2 W. 2.0 W is ~25x the noise floor and ~8x below the smallest
    measured genuine gap.
  - GPU: **3.0 W**. NVML `power.draw` at a 1500 MHz clock lock, `glxgears` proxy load: 0.11 W
    spread; a genuine idle-vs-loaded gap at the same lock, measured directly, was 6.1 W. 3.0 W is
    ~27x the noise floor and about half the smallest measured genuine gap — a larger margin
    relative to its own noise floor than the CPU's, because the GPU noise-floor number itself
    rests on a weaker (fill/geometry-bound, not compute-bound) proxy load; §Facts states that
    limitation explicitly.

  **Leaving the hold does not need a bespoke resync rule.** `Budget::step`'s existing generic
  "leaving any freeze" resync (`kind_switched || leaving_freeze`, in `src/control/budget.rs`,
  already implemented and untouched by this task) already fires exactly once whenever a
  `DemandLimited` tick is followed by a non-halted one, because `step` treats every
  `Some(freeze) -> None` transition uniformly. No new special case is required or was added.

  **`GPU HOT` does not freeze the integrator.** `Guards::gpu_share_override` (§2.8) already
  ratchets the GPU's effective `gpu_max_w` down toward its floor at `DOWN_RATE_W`/tick while hot;
  feeding that ratcheted value into `split_budget` as `gpu_max_w` is enough on its own —
  `split_budget`'s existing surplus-reassignment (an axis's rejected `cpu_over`/`gpu_over` flows
  to the other axis) automatically hands the GPU's shrinking share to the CPU axis, with no new
  mechanism. The demand-limited predicate above, evaluated against the resulting post-override
  `cap_i`, then behaves correctly on its own: neither axis shows a spurious gap during a HOT
  episode (confirmed by the sweep's scenario 6), so there is nothing for a `Budget`-level freeze
  to add.

  #### The sweep (candidate x scenario, `cargo test control::spike_antiwindup -- --nocapture`)

  Four candidates: `NoHalt` (baseline, no demand-limited halt at all — only the existing
  clamp+back-calc anti-windup), `Conditional` (the rule above, no hysteresis), the decided
  `ConditionalHysteresis`, and `CombinedSum` (directional, but judged on `Σdraw` vs `Σcap` rather
  than per axis — kept in the sweep specifically to demonstrate why the "judge each axis
  separately" invariant above is fixed, not optional). All four wrap the real `Budget`; the
  thermal plant is τ 35 s / θ 20 s / K 0.8 °C/W (§5), driven by **actual drawn watts**.

  | Scenario | NoHalt | Conditional | **ConditionalHysteresis (chosen)** | CombinedSum |
  |---|---|---|---|---|
  | 1 idle wind-up then load onset | **overshoots target by 14.0 °C-equiv** (windup) | holds, 0 overshoot | **holds, 0 overshoot** | holds, 0 overshoot |
  | 2 lull mid-session | **self-latches** (never recovers within the graded window) | recovers | **recovers** | recovers |
  | 3 warm-start, lighter load, EC above T* | holds | holds | **holds** | holds |
  | 4 mid-session target drop | holds | holds | **holds** | holds |
  | 5 structurally undrawn axis (dGPU unpowered) | holds | holds | **holds** | holds, but measurably worse than per-axis (see note) |
  | 6 GPU HOT, CPU at its own cap | holds (identical across all four — no differentiation expected, see below) | holds | **holds** | holds |
  | 7 oscillation around the margin | holds (never triggers a halt in this configuration) | holds, **81 hysteresis-state transitions** | **holds, 29 transitions** | holds (never triggers a halt in this configuration) |

  No candidate ever pulled `u` toward the draw in any cell (`cap_tracking` false throughout —
  `Budget::step`'s clamp-only back-calculation held in every run, as it must).

  **Scenario 5 note.** On this machine's specific measured `DEMAND_MARGIN_W` values, the GPU's
  structural floor share (5 W in the harness) sits close enough to `margin_sum` (2.0 + 3.0 = 5.0
  W) that `CombinedSum`'s closed-loop degradation versus per-axis is small in this configuration
  (mean settled error 0.063 °C-equiv vs 0.060 for both per-axis variants) rather than the more
  dramatic "CPU pinned at its floor" failure the invariant's reasoning describes — that failure
  is real and structural,
  not scenario-configuration-dependent, and is demonstrated directly (not just via this one
  closed-loop run) by two predicate-level tests in `spike_antiwindup.rs`:
  `per_axis_predicate_excludes_an_axis_pinned_at_its_floor` and
  `combined_sum_predicate_mis_flags_a_perfectly_tracked_cpu_next_to_a_pinned_gpu_floor` (the
  latter uses a deliberately larger floor than this harness's own 5 W specifically to make the
  point unambiguously, since the closed-loop run alone would understate it here).

  **Scenario 6 note.** All four candidates are identical because the scenario's target is
  reachable within `cpu_max_w + gpu_floor_w` even at the GPU's fully-ratcheted-down cap during the
  HOT episode (by construction — a target *not* reachable there would conflate the guard's own,
  expected, ceiling reduction with an anti-windup defect, which is not what this scenario tests).
  The identical result across candidates is itself the finding: none of them add a spurious extra
  hold on top of the guard's already-correct behavior.

  The harness (real `Budget`, thermal plant, demand model, the four `Rule` variants, the seven
  scenarios as data, and the predicate-level tests) is **kept** as a `#[cfg(test)]` fixture
  (`src/control/spike_antiwindup.rs`, declared in `src/control/mod.rs`) rather than deleted: it is
  the only place the decided rule and its constants are exercised against every named scenario
  together, and it stays green as a regression on both.

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
  point is that the design must not silently sit at the floor pretending to regulate, and §2.4's
  anti-windup is what must keep the integrator from winding while it cannot move the plant. That
  descent to the floor is one of the scenarios `fwloop.24` grades: a rule that blocks it is
  wrong, because the flag only fires once the bound is actually reached. Below ~64 °C the same
  curve is steep, so the run in §5 also checks that the loop does not hunt on the steep segment.
  **Mode B's gain schedule does not apply here.** `slope_at(T*)` describes the fw-fanctrl curve,
  which is not the plant in this regime — the EC's own curve is (§Facts). Whenever the socket
  reports `active: false` or `absent`, the schedule takes the same conservative `0.25×` clamp it
  uses for `None`, rather than scaling by a slope belonging to a curve that is not in the loop.
- **The same is true when the socket is `absent`, and for the same reason.** A stopped
  fw-fanctrl runs `--autofanctrl` from its unit's `ExecStopPost`, so a killed daemon leaves the
  fans on exactly the EC curve measured in §Facts. `absent` and `active: false` are therefore
  one plant regime, not two, and both are graded against the measured staircase; the difference
  between them is only which flag is raised.
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

**Scoring happens in the controller at the 1 Hz sample rate, not on the 5 s arbiter tick.** The
`fanctrl_view_changed` edge is produced once per new `print all` view by the 1 Hz sampler; if it
were consumed only when the arbiter runs, four of every five edges would be dropped, and because
the 30 s poll is an exact multiple of the 5 s tick the loss would be systematic rather than
random. The controller therefore evaluates the comparison on the sample carrying the view and
hands the arbiter the resulting counters; the arbiter owns the counters' meaning, not their
sampling.

On the first sample that carries a new `print all` view (the view's `all_observed_at` changed),
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
  DOWN_RATE_W)` each 5 s allocator tick; flag `GPU HOT`. **What the integrator does during an
  episode is deliberately unstated here and is one of `fwloop.24`'s decisions** — the override
  drives the GPU cap down to the present draw, so that axis is not demand-limited by any
  draw-versus-cap test, and if the CPU is simultaneously at its own cap (the sustained-gaming
  case this threshold is derived for) a naive "any axis at cap ⇒ integrate" rule winds `u`
  against watts the guard is discarding for the whole episode, then snaps them onto the GPU at
  exit. The spike settles whether the episode freezes the integrator or the discarded watts are
  handed to the CPU axis inside `split_budget`.
  **The default is derived, not guessed, but from one of four specs:** NVML publishes
  `GPU Target Temperature Specification` 87 °C, plus Shutdown, Slowdown and Max-Operating limits
  expressed as offsets against a T.Limit reference (measured 2026-09-08: current temp 38 °C with
  49 °C of T.Limit headroom, Shutdown −5, Slowdown −2, Max Operating 0, so the reference is that
  same 87 °C). 87 is where the card's own loop deliberately parks under sustained load, so a
  threshold below it latches for entire sessions and ratchets the GPU to its floor — which is why
  83/78 was wrong. 90/85 sits above the park point and below the slowdown region. Keying the
  guard on the published T.Limit margin instead of an absolute number is the more robust form and
  is left as a follow-on (§6).
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
  The first `Verified` clears the strike count, calls `resync_error` and unfreezes — **`u` is
  left untouched.** This is deliberately not a re-seed: §2.4 lists exactly three seeding events
  (auto entry, re-engagement from `Released`, calibration exit), and adding a fourth here would
  make an actuator hiccup silently reload a warm-start value. The RAPL stickiness watchdog stays
  as a second, independent check, unchanged, in `on_sample`.
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
`ModelDistrust`, adds `FanctrlLost`, `EcMismatch`, `SteepCurve`, `CurveInvalid` (warning),
`GpuHot`, `NvmeHot`, `ReadbackBlind`. `Effect::ModelSnapshot` is removed. Decision telemetry (`AutoAllocated`)
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
   Derive `λ = max(90, 3·θ̂)` and `Kc = τ / (K·(λ + θ̂))`, `Ti = τ` for each, **using the fitted
   `θ̂` exactly as measured**. There is no `+ ma_interval/2` here: this fit runs on the already-
   filtered EC average, so `θ̂` already contains the boxcar, and adding it again would count the
   filter twice and roughly halve `Kc`. The `θ_eff` substitution belongs only to §2.4's
   raw-domain defaults.
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

- `sensors/ec.rs` (2.2) at the 1 Hz sampler tick. **The NVMe reader does not run there**: a
  drive temperature read is a SMART admin command whose kernel `admin_timeout` defaults to 60 s,
  and putting an un-timed one on the thread that produces every `Sample` would let a stalling
  drive stall the control loop — for a value that now only feeds a flag (§2.8). **Nor does it go
  on the `FanctrlPoller` thread**, which owns the 5 s `print speed` poll whose 15 s staleness
  rule decides the loop's mode: a 60 s admin stall there would fake a socket outage and drop the
  loop out of Mode A. It gets **its own thread**, publishing a stamped last-good value that a
  missing or stale reading turns into `None`, which already deactivates the guard. `Sample` gains
  `ec: Option<EcReading>`, `ec_valid`, `nvme_temp_c: Option<f64>`, `fanctrl: Option<FanctrlView>`,
  `on_ac: bool` (from `/sys/class/power_supply/ACAD/online`). `cpu_pkg_w` (RAPL) and `gpu_w`
  (NVML) are already sampled and now also feed the per-axis `Budget::demand_limited` check
  (§2.4); the existing
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
- Demand-limited behaviour, the shape roast iteration 2 found broken twice: a duty-cycled load
  (5 min on, 2 min off, repeated) must not let `u` decay toward the lull's draw, and the load
  must come back inside ±150 RPM within 90 s of each onset; a CPU-only run with the dGPU
  unpowered must leave `u` off its lower bound and the CPU cap above its floor after 10 min.
- Socket `absent` with the EC-autofan plant (the killed-daemon regime, §2.5): the loop reports
  `TARGET UNREACHABLE (low)` rather than winding, and `FANCTRL LOST` is raised.
- A rejected non-monotone curve: TempLoop never engages, `CURVE INVALID` is raised at warning
  severity, and RpmLoop runs with the conservative `0.25×` gain clamp.
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
- **Key the dGPU guard on NVML's published T.Limit margin** rather than an absolute 90 °C, so it
  tracks the card's own thermal reference instead of a number measured once.
- **Measure the EC autofan curve's rising branch and hysteresis** (slow 55 → 75 °C ramp with a
  separate decay, 1 Hz) if the `active: false` / `absent` regime ever needs more than the
  qualitative "no authority on the plateau" result the current plant encodes.
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
| fwloop.1 | 1 | — | Curve model + DutyRpmTable | New `src/fanctrl/{mod,curve,table}.rs`. `Curve::from_points(Vec<(f64, u8)>) -> Result<Curve>` in file order, **rejecting any descending segment** (§2.1: a non-monotone curve makes `tread` ill-posed and hides steepness; the rejection surfaces as `CURVE INVALID` at warning severity via fwloop.10's `curve_valid` input, never as `SteepCurve`); `duty_at`, `tread`, `t_star`, `slope_at`, `nearest_tread(d) -> Option<u8>` (the §2.3 fallback: nearest duty with a tread, preferring lower; `None` when no tread exists at or below `d`), `min_tread_duty()`. `DutyRpmTable` with `Default` = the ten seeded points (also the serde default), `duty_for_rpm` (ties down), `rpm_for_duty`, `refine(duty, mean_rpm)` EWMA 0.8/0.2 rejected when `|mean − rpm| > 25 %` and clamped to keep the table strictly increasing in duty. Unit tests use the §Facts point lists directly (no fixture dependency). **owns:** `Curve` API including `nearest_tread`/`min_tread_duty`, `DutyRpmTable` API, its `Default` and serde shape, the refinement invariants. Files: `src/fanctrl/curve.rs`, `src/fanctrl/table.rs`, `src/fanctrl/mod.rs`, `src/main.rs` (mod decl). Acceptance: on the `quiet16`/`cool16` points — treads, T*, slopes, `nearest_tread` for a skipped integer resolves to the nearest lower tread and to `None` below the floor; a curve with a descending segment is rejected; `default()` equals the seed and a JSON without the field deserialises to it; interpolation, snap ties; a refinement that would invert two adjacent duties is clamped and the table stays monotone after 100 noisy refinements; a > 25 % jump is rejected. | [cov-1] · [cov-2] |
| fwloop.2 | 1 | fwloop.20 | fw-fanctrl socket client | `src/fanctrl/client.rs`: `PrintCommand { Speed, All }` (the only commands the client can send), `FanctrlSource` trait + `UnixFanctrlClient` (connect 1 s, read 3 s, raw CLI string, read to EOF) + `FakeFanctrl` in `src/test_support/fakes.rs` that records every command and replays scripted views/failures. `FanctrlView` (§2.1) with `observed_at` (any poll) and `all_observed_at` (last `print all`), `curve` as the raw `Vec<(f64, u8)>` from `resolve_curve(print_all_json, strategy)` (exact-name match in `strategies`); `Freshness { Fresh, Stale, Absent }` with the 15 s (`print speed`) / 90 s (`print all`) rules. Config key `fanctrl_socket` (default `/run/fw-fanctrl/.fw-fanctrl.commands.sock`). **owns:** `PrintCommand`, `FanctrlSource` trait, `FanctrlView` struct and its two stamps, `resolve_curve` (strategy resolution), `Freshness` and its timing rules, `fanctrl_socket` config key, `FakeFanctrl`. **consumes:** socket fixtures and the fakes module slot (fwloop.20). blocked-by fwloop.20: consumes the `print all`/`print speed` fixtures and `src/test_support/fakes.rs`. Files: `src/fanctrl/client.rs`, `src/fanctrl/mod.rs` (mod decl only), `src/config.rs`, `src/test_support/fakes.rs`. Acceptance: parses the fixtures (strategy, active, speed, temperature, ma interval); `resolve_curve` yields exactly the §Facts points for `quiet16` and `cool16`; ENOENT → Absent; read timeout → Stale; `print speed` failing for 15 s while `print all` is fresh → Stale; a `print speed` refresh bumps `observed_at` but not `all_observed_at`; the fake's command log contains only `Speed`/`All`. | [cov-1] · [cov-2] |
| fwloop.3 | 1 | fwloop.20 | EC replica + NVMe + AC sensors | `src/sensors/ec.rs`: `EcReading`/`EcLabel` (controllable `apu`, `cpu`, `gpu_*` if one ever reports; uncontrollable `ambient`, `charger`), every positive reading takes part in the max, drop ≤0 / unreadable / unparsable `_input`, round to integer, max + argmax (ties: sysfs order); `EcAverage` boxcar of N non-zero samples with the off-by-one, `set_interval(n)` cap 100 **retaining** existing samples, `reseed(value)` the only clearing operation, plus `is_seeded()`/`sample_count()` so the controller can refuse to use an underfilled mean as a full one. `sensors/hwmon.rs` gains `nvme_composite_c() -> Option<f64>` and `on_ac()`. **owns:** `EcReading`, `EcLabel` and its controllability rule, `EcAverage` and its retain-on-resize contract, `nvme_composite_c`, `on_ac`. **consumes:** hwmon/power_supply fixtures (fwloop.20). blocked-by fwloop.20: consumes the two cros_ec trees, nvme and ACAD fixtures. Files: `src/sensors/ec.rs`, `src/sensors/hwmon.rs`, `src/sensors/mod.rs`. Acceptance: on `cros_ec_idle` 47.85 → 48, argmax `ambient`, the −150 and the input-less sensor dropped without invalidating the reading; on `cros_ec_load` 74.85 → 75 with argmax `cpu@4c`, matching that fixture's paired socket `temperature` of 75.0; on `cros_ec_dgpu_on` the `gpu_*` sensors are still −150/absent and are dropped, so the max comes from `cpu`/`ambient` (measured: they never report even with the dGPU powered); a synthetic positive `gpu_*` reading would join the max and classify controllable; boxcar returns mean of n−N..n−1; `set_interval` grows/shrinks without clearing, `reseed` replaces the contents; `is_seeded` is false until seeded or N samples deep; nvme/ac readers on fixtures, nvme `None` when the chip is absent. | [cov-1] · [cov-2] · [roast-1] |
| fwloop.4 | 1 | — | Budget integrator | `src/control/budget.rs`: `Budget::new(&LoopGains)`, `set_gains`, velocity-form PI, `PI_PERIOD_S=5`, `LoopError { Temp{e_c}, Rpm{e_rpm} }`, `Freeze { ActuatorMismatch, Calibrating, Released, DemandLimited }`, clamp + back-calculation `Tt=Ti` **against the bounds only** (§2.4 — there is deliberately no back-calculation toward the measured draw; that shape was tried and produces a cap that tracks the draw). **The demand-limited predicate itself is not this task's to invent** — `fwloop.24` settles it and writes it into §2.4; this task exposes the seam it plugs into: `set_demand_state(&[(draw, cap)], error_sign)` returning whether accumulation is halted, honouring §2.4's fixed invariant that a halt may only ever block the direction that deepens the condition, never the recovering one. `set_bounds(lo,hi)`, `seed(u)`, `resync_error(e)` (resets e_{k−1} without touching u), `step(err, freeze) -> f64`, an error-kind switch and leaving any freeze both call `resync_error` implicitly; `at_lower_bound_for()` and `at_upper_bound_for() -> Duration` for the two unreachable rules; `scale_rpm_gain(slope: Option<f64>)` applying the §2.4 curve-slope schedule, with `None` ⇒ the conservative `0.25×` clamp. `LoopGains` struct with serde and `Default` = the θ_eff-derived IMC defaults of §2.4 (`kc_w_per_c = 0.22`, `ti_s = 35`, `kc_w_per_rpm = 0.0028`, `ti_rpm_s = 35`). `WarmStart` map API: `key(strategy,duty,on_ac) -> String`, `lookup`, `record`. **owns:** `Budget`, `LoopError`, `Freeze`, `LoopGains`, `WarmStart` types. Files: `src/control/budget.rs`, `src/control/mod.rs`. Acceptance: step response on a first-order plant reaches within 1 % with overshoot ≤ 5 % at defaults; a non-default `LoopGains` changes the step magnitude; clamp holds at bounds without wind-up (release recovers within one Ti); **a halt blocks only the deepening direction — with the condition active and the error calling for less heat, `u` still integrates down** (the latch that failed roast iteration 3), and `u` never decays toward the draw; freeze holds u exactly; `resync_error` after a setpoint jump and on leaving a freeze produce no proportional kick; a Temp→Rpm switch produces |Δu| ≤ one integral increment; `at_lower_bound_for`/`at_upper_bound_for` count only while clamped at their own bound; `scale_rpm_gain` returns 1× at `slope_ref`, 0.25× at four times `slope_ref`, and 0.25× on `None`. | [cov-1] · [cov-2] · [roast-1] · [roast-2] |
| fwloop.5 | 1 | — | Allocator: scalar budget split | `src/control/allocator.rs`: `AllocInput { budget_w, demand, floors, cpu_max_w, gpu_max_w, gpu_floor_w }`; `split_budget` (floors first, ∝ demand, surplus to the other axis, `GRID_STEP_W`); `UP_RATE_W`/`DOWN_RATE_W` slew clamp retained; delete deadband/raise-hold/slope-gate/overshoot drain/veto/taper/`CONSERVATIVE_START`/`overshoot_settle_*`, their constants, and the three field-replay sims (`simulate_field_cycle`, `simulate_soak_cycle`, `simulate_ec_overshoot_cycle`) — `control/trim.rs` becomes unused and is deleted by fwloop.14. `allocator::demand` unchanged. The controller edit is confined to the `allocator.step` call site, compiling against the new shape with a placeholder budget (sum of floors) until fwloop.12. **owns:** `AllocInput` shape, `split_budget`. Files: `src/control/allocator.rs`, `src/control/mod.rs`, `src/control/controller.rs` (the `allocator.step` call site only). Acceptance: floors always met; both axes capped with surplus reassigned; equal split at zero demand; slew clamp bounds per-tick change; no reference to `contour`, `CONSERVATIVE_START` or `overshoot_settle` remains under `src/control/allocator.rs` or the call site (the repo-wide sweep is fwloop.14's). | [cov-1] · [cov-2] |
| fwloop.6 | 1 | — | Guards (dGPU, NVMe) + config keys | `src/control/guards.rs`: `Guards::step(gpu_temp_c: Option<f64>, nvme_temp_c: Option<f64>) -> GuardState { gpu_hot, nvme_hot }` with enter thresholds, exit = enter − 5, and `None` ⇒ that guard inactive (exits any hot state); `gpu_share_override(current_gpu_w, gpu_floor_w)`. **There is no `effective_target`:** the NVMe guard is reporting-only (§2.8, measured), so nothing modifies the user's RPM target and `nvme_hot` only drives a flag. Config keys `gpu_hot_c` (**90**, exit 85 — derived from the card's own 87 °C target, §2.8), `nvme_hot_c` (80); the `online_rls` legacy note/test in `config.rs` generalised to "unknown keys are ignored". **owns:** `Guards`, `GuardState`, the absent-reading rule, the two config keys, the unknown-key tolerance rule. Files: `src/control/guards.rs`, `src/config.rs`, `src/control/mod.rs`. Acceptance: hysteresis enters at threshold, exits 5 below; a `None` input deactivates the guard and clears a hot state; the GPU share override is computed per spec §2.8; a hot NVMe sets `nvme_hot` and changes nothing else (no target, no budget); config round-trips with defaults; a config containing `online_rls`, a stale `nvme_boost_rpm` and an arbitrary unknown key still loads. | [cov-2] · [roast-1] |
| fwloop.7 | 1 | fwloop.20 | Actuator read-back | `src/actuators/cpu.rs`: write `--slow-limit --stapm-limit --fast-limit`, then run `ryzenadj --info` through the `Runner`, parse `PPT LIMIT SLOW`/`PPT LIMIT FAST`/`STAPM LIMIT` rows; return `WriteVerdict { Verified(w), Mismatch{field,commanded,read}, Unreadable, Unverifiable }` (slow/fast within 0.5 W; STAPM not required; a failed `--info` ⇒ `Unreadable`, never `Mismatch`). `src/actuators/gpu.rs`: `verify_lock(gpu_util, gpu_sm_mhz) -> WriteVerdict` using the LUT-sweep pin rule (util > 90 %, sm ≤ locked + 30, 3 samples). Existing controller callers compile by treating non-`Verified` as today's success/failure until fwloop.12; the controller edit is confined to the two actuator call sites; the RAPL stickiness watchdog in `on_sample` is untouched. **owns:** `WriteVerdict`, `set_sustained_mw` return type, `verify_lock`. **consumes:** the `ryzenadj --info` fixture (fwloop.20). blocked-by fwloop.20: consumes `ryzenadj_info.txt`. Files: `src/actuators/cpu.rs`, `src/actuators/gpu.rs`, `src/control/controller.rs` (the two call sites only). Acceptance: parser on the fixture table; mismatch detected on a fake runner returning a stale table; a runner whose `--info` fails yields `Unreadable`; GPU verdict `Unverifiable` under 90 % util and `Mismatch` when the pinned clock exceeds lock + 30 for 3 samples; the existing RAPL watchdog tests still pass. | [cov-1] · [cov-2] |
| fwloop.8 | 1 | — | Controller status surface | `src/control/controller.rs` types only: `LoopMode { TempLoop, RpmLoop, Released }`; `ControlStatus` drops `trim_rpm`/`gain`, adds `mode`, `t_star_c: Option<f64>`, `ec_ma_c: Option<f64>`, `ec_argmax: Option<String>`, `duty_cmd: Option<u8>`, `snapped_rpm: f64`, `strategy: Option<String>`, `budget_w: f64`; `CalibProgressLite.phase` stays a plain `String`; `StatusFlag` drops `ModelDistrust`, adds `FanctrlLost`, `EcMismatch`, `SteepCurve` (info), `CurveInvalid` (warning — a permanent loss of Mode A must not share the informational `SteepCurve`), `GpuHot`, `NvmeHot`, `ReadbackBlind` with severities; `Effect::ModelSnapshot` removed; `Effect::AutoAllocated` gains `mode`, `error: f64`, `budget_w`, `freeze: Option<&'static str>` (all plain types — this task stays dependency-free). Existing code populates the new fields with defaults so the crate compiles; UI/telemetry call sites updated minimally (compile-only). **owns:** `LoopMode`, `ControlStatus` field set, `StatusFlag` set, `Effect` variants. Files: `src/control/controller.rs`, `src/ui/view.rs` (compile-only), `src/telemetry.rs` (compile-only). Acceptance: crate compiles and tests pass with the new type surface; `flag_severity` covers every new flag; the new `ControlStatus`/`Effect` definitions carry no `trim_rpm`/`gain`/`ModelSnapshot` field or variant. The repo-wide symbol sweep is **not** this task's — `controller.rs` still holds the adaptation tier and its tests at this point, and removing them is fwloop.22's chartered work; this task only reshapes the type surface and updates call sites enough to compile. | [cov-1] · [cov-2] |
| fwloop.22 | 1 | fwloop.8 | Remove the adaptation tier | `src/control/controller.rs`: delete the five-gate adaptation tier, the cooldown ring, the trust monitor, the `ModelSnapshot` period, the degrade guard's `model.is_none()` check, `persisted_bias`/`persisted_gain` plumbing, and every controller test that exercises them (the KF/trust/cooldown/`fitted_model` blocks). The auto path compiles with the budget stubbed at the floors and the existing `AllocInput` call site; the crate builds and the surviving tests pass. **owns:** the removal of the adaptation tier and its tests. **consumes:** the `ControlStatus`/`Effect` surface without `trim_rpm`/`gain`/`ModelSnapshot` (fwloop.8). blocked-by fwloop.8: consumes the model-free status surface. Files: `src/control/controller.rs`. Acceptance: no import of `thermal_model`, `kalman`, `trust`, `cooldown` remains in the controller (source or tests); no `adapt_bias`/`adapt_gain`/`persisted_bias` symbol remains; `cargo test` green. | [cov-2] split from fwloop.12 |
| fwloop.9 | 1 | fwloop.2, fwloop.3 | Sample plumbing + socket poller | `src/types.rs` `Sample` gains `ec: Option<EcReading>`, `ec_valid`, `nvme_temp_c: Option<f64>`, `fanctrl: Option<FanctrlView>`, `fanctrl_freshness`, `fanctrl_view_changed: bool` (true on the first sample whose view `all_observed_at` differs from the previous sample's — a speed-only refresh never sets it), `on_ac`, and `resumed` (already produced by the existing resume handler, now also consumed by the EC boxcar and steady-window invalidation of §2.2). `src/sensors/sampler.rs` (1 Hz tick) reads EC and AC each tick and merges the latest view from a background `FanctrlPoller` thread (`print speed` every 5 s, `print all` every 30 s, never faster; shares an `Arc<Mutex<...>>`); **the NVMe temperature is polled at 30 s on a thread of its own — neither the sampler tick nor the `FanctrlPoller`** (§3.4 — a SMART admin read can block for the kernel's 60 s `admin_timeout`; on the sampler it would stall the control loop, and on the poller it would fake a socket outage through the 15 s `print speed` staleness rule and drop the loop out of Mode A), publishing a stamped last-good value, with a missing or stale value surfacing as `None`; `src/main.rs`: the poller is constructed from `Config::fanctrl_socket` with the real `UnixFanctrlClient` and its thread is joined on shutdown. All freshness/cadence decisions are computed from the `Sample`/`FanctrlView` monotonic timestamps; the poller's own `Instant` drives only its cadence. **owns:** the new `Sample` fields including `fanctrl_view_changed`, `FanctrlPoller` cadence, poller construction and shutdown, the timestamp-not-wall-clock rule. **consumes:** `EcReading`/`nvme_composite_c`/`on_ac` (fwloop.3), `PrintCommand`/`FanctrlSource`/`FanctrlView`/`Freshness` (fwloop.2). blocked-by fwloop.2: consumes `FanctrlSource` trait, `PrintCommand`, `FanctrlView` (two stamps), `Freshness`. blocked-by fwloop.3: consumes `EcReading`, `nvme_composite_c`, `on_ac`. Files: `src/types.rs`, `src/sensors/sampler.rs`, `src/sensors/mod.rs`, `src/main.rs`. Acceptance: with the fake source, over a 60 s scripted run the poller issues `Speed` at 5 s ±1 tick and `All` exactly twice; `Stale` after 90 s of `All` failures and after 15 s of `Speed` failures; `fanctrl_view_changed` is true exactly once per new `All` view and never on a `Speed`-only refresh; **a scripted NVMe read that blocks for 60 s delays no `Sample`, leaves `Freshness` `Fresh` and the `print speed` cadence intact, and surfaces only as `nvme_temp_c: None`**; the real client is constructed from the configured path and both threads join on shutdown. | [cov-1] · [cov-2] · [roast-2] · [roast-3] |
| fwloop.10 | 1 | fwloop.1, fwloop.2, fwloop.3, fwloop.8 | Mode arbiter, reconciliation, feasibility | `src/control/mode.rs`: `Arbiter::decide(&ArbiterInput) -> Decision { mode, t_star, slope, reasons, flags, reseed_ma: Option<f64>, ec_mismatch: bool, t_star_changed: bool }` implementing the §2.5 table, 3-tick entry hysteresis, immediate exit on hard faults, `EC MISMATCH` counters (§2.6; `view_changed: bool` in the input is a plain flag supplied by the controller) **including the skip rule** (a view is scored only when the replica's 5 s slope is under 0.5 °C/s and the view-to-sample stamp gap is under 2 s) **and the moving-average check** (|`ec_ma` − `ma_temperature`| ≤ 2 °C, three failures requesting a re-seed rather than latching a mismatch), feasibility + steepness (§2.7) with the 60 s feasible-again clear, the **low** unreachable rule (`target_duty` below `min_tread_duty()`, or `at_lower_bound_for ≥ 60 s` with the error still negative), the symmetric **high** rule (`at_upper_bound_for ≥ 60 s` with the error still positive), the **debounced** argmax-controllable condition (3 consecutive failures or a > 1 °C lead before it can drop TempLoop), `FANCTRL LOST` clearing on the first fresh view, and re-derivation of T* whenever the view's curve **points** differ from the cached ones (cache keyed on points, not the strategy name) or `target_duty` changes — `t_star_changed` tells the controller to `resync_error`. `ArbiterInput` carries `fanctrl: Option<&FanctrlView>` + `Freshness` + `view_changed`, `ec: Option<&EcReading>`, `ec_ma: Option<f64>`, `fan_valid`, `target_duty`, `at_lower_bound_for`, **`at_upper_bound_for`**, `error_sign`, **`curve_valid: bool`** (false when `from_points` rejected a non-monotone curve), and the reconciliation counters the controller scores at 1 Hz (§2.6 — this row owns their meaning, not their sampling rate). `Decision.slope` is an `Option<f64>`, `None` when no curve is resolved. **owns:** `Arbiter`, `ArbiterInput`, `Decision`, mismatch/feasibility counters, the point-keyed cached `Curve`, all T* derivation. **consumes:** `Curve::from_points/tread/t_star/slope_at/min_tread_duty` (fwloop.1), `FanctrlView`/`Freshness` (fwloop.2), `EcReading`/`EcLabel` (fwloop.3), `LoopMode`/new `StatusFlag`s (fwloop.8). blocked-by fwloop.1: consumes `Curve::from_points`, `tread`, `slope_at`, `min_tread_duty`. blocked-by fwloop.2: consumes `FanctrlView` + `Freshness`. blocked-by fwloop.3: consumes `EcReading`, `EcLabel` controllability. blocked-by fwloop.8: consumes `LoopMode`, `StatusFlag` variants. Files: `src/control/mode.rs`, `src/control/mod.rs`. Acceptance: table-driven tests for every row of §2.5; 3-tick hysteresis in, immediate out; three mismatches → RpmLoop, three matches → TempLoop with `reseed_ma`; infeasible target yields `TargetUnreachable` with the explanatory text and clears only after 60 s of continuous feasibility; a sub-floor duty and a 60 s low-clamp each yield the `low` reason; **a target above the fans' reach holds the upper bound 60 s and yields the `high` reason**; `FanctrlLost` clears on the first fresh view; a cool16 tread above 70 → `SteepCurve`; **`curve_valid: false` yields RpmLoop with `CurveInvalid` and `slope: None`, never `SteepCurve`**; the initial state reports the `unreconciled` reason, distinct from a mismatch; an edit of the active strategy's points under the same name re-derives T* and sets `t_star_changed`. | [cov-1] · [cov-2] · [roast-2] |
| fwloop.11 | 1 | fwloop.1, fwloop.4, fwloop.20 | Persisted state migration | `src/state.rs`: `PersistedState { lut, calibrated_at, loop_gains: Option<LoopGains>, duty_rpm_table: DutyRpmTable, warm_start: BTreeMap<String,f64> }`; remove `model`, `adapt_bias`, `adapt_gain` and the `state.rs` thermal-model fit tests; old files load (unknown keys ignored, missing new keys default). The controller edit is confined to the persist call sites (`save_persisted_state`, `exit_auto_and_persist`, `apply_calib_effects`). **owns:** the `state.json` schema. **consumes:** `LoopGains`/`WarmStart` (fwloop.4), `DutyRpmTable` serde + `Default` (fwloop.1), `state_v1.json` (fwloop.20). blocked-by fwloop.1: consumes `DutyRpmTable` serde shape and `Default`. blocked-by fwloop.4: consumes `LoopGains` serde shape. blocked-by fwloop.20: consumes the `state_v1.json` fixture. Files: `src/state.rs`, `src/control/controller.rs` (persist call sites only). Acceptance: the `state_v1.json` fixture loads with `lut` intact, `duty_rpm_table` equal to the ten seeded points, `warm_start` empty and `loop_gains` `None`; round-trip of the new schema; no `thermal_model` import remains in `state.rs`. | [cov-1] · [cov-2] |
| fwloop.24 | 1 | fwloop.4 | Spike: settle the anti-windup rule | **A spike, not production code** — its output is a decision written into §2.4, and its harness is either deleted or kept as a `#[cfg(test)]` fixture, whichever the result argues for. Build a throwaway rig around the real `Budget`: a first-order thermal plant (τ 35, θ 20, K 0.8 per §5) plus a demand model that decides how much of each commanded cap is actually drawn, so a cap can sit above the draw. Sweep the candidate rules — no halt at all; conditional integration halting only the deepening direction; the same with hysteresis/dwell; per-axis versus combined — across **every scenario the three roast rounds named**: idle wind-up to the ceiling then a load onset; a lull mid-session; a warm-start seeded from a heavier session with a lighter load and the EC above T*; a mid-session target drop; a structurally undrawn axis (dGPU unpowered, CPU-only load); a `GPU HOT` episode with the CPU at its own cap; and oscillation around the margin when the draw sits near the cap. **owns:** the decision, its constants, and the §2.4 rewrite that records them. **consumes:** `Budget` and its `set_demand_state` seam (fwloop.4). blocked-by fwloop.4: consumes the `Budget` type and the halt seam the spike selects the rule for. Files: `src/control/spike_antiwindup.rs` (cfg(test), throwaway), `docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md` (§2.4). Acceptance: every scenario above is run under every candidate and the results tabulated in the spec; the chosen rule holds the fan target in all of them with no self-latch and no cap-tracking; `DEMAND_MARGIN_W` is set per axis from the measured commanded-versus-drawn spread (RAPL against the slow-limit, NVML watts against the clock lock) rather than assumed, and that measurement is recorded in §Facts; §2.4's "open for the spike to decide" list is replaced by the decided rule. | [roast-3] promoted from prose |
| fwloop.12 | 1 | fwloop.1, fwloop.3, fwloop.4, fwloop.5, fwloop.6, fwloop.7, fwloop.8, fwloop.9, fwloop.10, fwloop.11, fwloop.22, fwloop.24 | Controller loop integration | `src/control/controller.rs`, on the tier-free controller from fwloop.22: `Budget::new(persisted loop_gains or default)` on auto entry; `on_auto_sample` = window pushes (existing fan window; `rpm_smoothed` = `FAN_SMOOTH_N` tail-mean, `fan_valid` from the sample) + `EcAverage` push (controller owns the live instance: `set_interval` on view change, `reseed` on `Decision.reseed_ma`) → guards (`Option` inputs; `nvme_hot` raises its flag only) → every 5 s: budget bounds (`lo = cpu_floor_w + lut.watts_at(gpu_floor_mhz)`, `hi = cpu_max_w + gpu_max_w`, `Budget::set_bounds`) → `target_duty` from the user's target via `DutyRpmTable::duty_for_rpm` + `Curve::nearest_tread` → arbiter (`ArbiterInput` incl. `view_changed`, `at_lower_bound_for`, `at_upper_bound_for`, `error_sign`) → `resync_error` when `t_star_changed` → `LoopError` (Mode B error against `rpm_for_duty(target_duty)`, gain scaled by `scale_rpm_gain(slope)`) → **the anti-windup halt as `fwloop.24` decided it** (`Budget::set_demand_state` with the per-axis smoothed draws, their caps and the error sign; the spike also fixes whether the cap is the pre- or post-guard-override value, and what a `GPU HOT` episode does) → `split_budget` (+ guard overrides + slew clamp) → CPU write + read-back verdict → GPU PI target at 1 Hz + `verify_lock` verdict; both verdicts share one rule: a candidate `Mismatch` is re-read once and suppressed for 3 ticks after an `on_ac` edge, then `Mismatch` ⇒ `LimitNotSticking` + `Freeze::ActuatorMismatch` + immediate reassert on the same tick, three consecutive ⇒ release to stock with the flag held **while the write + read-back keeps running every reassert period, so a later `Verified` is producible** and clears the strikes; `Unreadable`/`Unverifiable` are non-events, six consecutive `Unreadable` ⇒ `ReadbackBlind` until the next `Verified`. `EcAverage` is seeded from `view.ma_temperature` on auto entry, on re-engagement and at calibration exit, and it plus the fan and steady windows are cleared on a `resumed` sample. `Released` decision ⇒ `release_to_stock`, `Freeze::Released`, flags; a later usable decision re-engages (seed from the floors here; warm-start arrives in fwloop.19). Auto entry requires `lut` only; transitions emit `Noted{mode:...}`; status fields mirrored every tick; the RAPL stickiness watchdog in `on_sample` is retained untouched. **owns:** the auto-mode data flow, the live `EcAverage` instance, gains loading, the budget-bounds derivation (exposed later as `CalibContext.budget_bounds`), the target → `target_duty` snap incl. the no-tread fallback, the RPM window/`fan_valid` supply, the `resync_error` call sites, the verdict rule for both actuators, the `Released` branch, the per-axis demand-limited evaluation, the 1 Hz reconciliation scoring (§2.6) whose counters the arbiter consumes. **consumes:** everything listed in deps. blocked-by fwloop.1: consumes `DutyRpmTable::duty_for_rpm/rpm_for_duty`, `Curve::nearest_tread`. blocked-by fwloop.3: consumes `EcAverage::{push, set_interval, reseed}` and its boxcar contract. blocked-by fwloop.4: consumes `Budget` (`new`, `set_bounds`, `resync_error`, `step`, `demand_limited`, `scale_rpm_gain`, `at_lower_bound_for`, `at_upper_bound_for`), `LoopError`, `Freeze`. blocked-by fwloop.5: consumes `AllocInput{budget_w}`, `split_budget`. blocked-by fwloop.6: consumes `Guards::step` and the GPU share override. blocked-by fwloop.7: consumes `WriteVerdict`, `verify_lock`. blocked-by fwloop.8: consumes `LoopMode`, `ControlStatus` fields, new flags. blocked-by fwloop.9: consumes `Sample.ec/fanctrl/fanctrl_freshness/fanctrl_view_changed/nvme_temp_c/on_ac`. blocked-by fwloop.10: consumes `Arbiter::decide` → `Decision`. blocked-by fwloop.11: consumes `PersistedState.loop_gains/duty_rpm_table/warm_start`. blocked-by fwloop.22: consumes the tier-free controller. blocked-by fwloop.24: consumes the decided anti-windup rule and its constants — the wiring cannot be written before the rule exists. Files: `src/control/controller.rs`. Acceptance: controller unit tests on the fake seams cover: auto entry with LUT only; `Some(gains)` is loaded into `Budget`, `None` uses defaults; the integrator floor tracks a LUT change; an `NVME HOT` tick raises the flag and leaves `target_duty`, T* and the budget **unchanged**; a tick whose snapped duty the curve skips uses `nearest_tread`; a TempLoop tick computes `T* − MA` and moves the budget; an RpmLoop tick on socket Absent drives u from `rpm_for_duty(target_duty) − rpm_smoothed`; a same-name curve edit calls `resync_error` and produces no kick; a fan dropout clears `fan_valid` within one window; **the anti-windup scenarios `fwloop.24` decided, replayed at controller level against the rule it chose** — at minimum: an idle tick far below the cap neither winds to the ceiling nor decays toward the draw; a seeded-high budget with a lighter load still integrates **down**; a CPU-only tick with the dGPU unpowered keeps `u` off its lower bound; and a `GPU HOT` episode behaves as the spike specified; a scored view is evaluated on the 1 Hz sample carrying it, not on the 5 s arbiter tick; a rejected curve raises `CurveInvalid` and RpmLoop runs at the `0.25×` clamp; a CPU and a GPU `Mismatch` each freeze, flag and reassert on the same tick, and a mismatch within 3 ticks of an `on_ac` edge is suppressed; three consecutive mismatches release to stock with the flag held, the read-back keeps running, and a later `Verified` re-engages without a step; six `Unreadable` raise `ReadbackBlind` with no freeze; a `resumed` sample clears the boxcar and the steady window; a `Released` decision releases the caps, freezes, and a later usable decision re-engages without a step in u; a view change updates the boxcar interval without discontinuity in `ec_ma_c` and a `reseed_ma` decision re-seeds it; `Noted` transitions appear; the RAPL watchdog test still passes. | SPLIT applied (→19) · [cov-1] · [cov-2] split (→22) |
| fwloop.21 | 1 | fwloop.4 | FOPDT fit + IMC gain derivation | `src/calib/fopdt.rs`: `fit_fopdt(&[(t, y)], step_w) -> Option<Fopdt{k,tau,theta}>` by least squares on a step response; `derive_gains(ec: &Fopdt, rpm: &Fopdt, defaults: &LoopGains) -> Option<LoopGains>` with **θ taken as fitted** — the §3.3 fit runs on the already-filtered EC average, so its `θ̂` already contains the boxcar and adding `ma_interval/2` again would count the filter twice and roughly halve `Kc`; the `+ma_interval/2` substitution belongs only to the raw-domain defaults of §2.4 — then λ = max(90, 3θ̂), `Kc = τ/(K(λ+θ̂))`, `Ti = τ` per signal, `fitted_at` left `None` for the caller to stamp; rejection rules — `K ≤ 0`, `τ < 5 s`, a response magnitude under 3 °C (EC) or 150 RPM (fan), or a derived `Kc` outside [0.25, 4]× the corresponding default — all yielding `None`. The magnitude and ratio bounds are what stop `Kc = τ/(K(λ+θ))` running away as `K → 0⁺`. Pure functions, no I/O. **owns:** `Fopdt`, `fit_fopdt`, `derive_gains`. **consumes:** `LoopGains` (fwloop.4). blocked-by fwloop.4: consumes the `LoopGains` struct. Files: `src/calib/fopdt.rs`, `src/calib/mod.rs`. Acceptance: recovers K/τ/θ within 10 % on a synthetic noisy FOPDT step; derived gains match the IMC formulas **with the fitted `θ̂` used as-is, no `+ ma_interval/2`** (the boxcar is already in `θ̂`; adding it again halves `Kc`); each rejection rule fires on its own crafted input — `K ≤ 0`, `τ < 5 s`, a sub-threshold response, and a duty-pinned step whose tiny positive `K_rpm` would otherwise derive a `Kc` orders of magnitude above the default. | [cov-1] split from fwloop.13 · [roast-1] |
| fwloop.13 | 1 | fwloop.4, fwloop.11, fwloop.21 | Calibration step test | `src/calib/runner.rs`: phases `LutSweep → StepTest → Done`; delete `MATRIX_POINTS`, `MatrixPoint`, `Fitting`, `record_matrix_point`, the `fit_batch` use and their tests; `on_sample(&Sample, &CalibContext)` where `CalibContext: Default`; new `src/calib/step.rs`: **burner started first**, then settle detection (EC MA flat ≤ 0.5 °C over 60 s and RPM steady, cap 5 min), then a `budget_bounds.0 + 30 W` step requested via the new `RunnerEffect::SetBudget(w)`, `NeedsLoad` nagged while GPU utilisation is below the pin threshold, 5 min record of EC MA, RPM and **measured** applied power per axis, then `fit_fopdt` + `derive_gains` on the measured per-axis power delta (never the nominal 30 W), stamping `fitted_at` from the sample clock. Two-stage gating: `fanctrl_active` and `!ec_mismatch` before the settle, `argmax_controllable` **after the burner is running** (at the floors the machine's own idle fixture has ambient above apu, so an up-front check self-skips every run). Skips with a `Noted` reason when either gate fails, when the applied power never rose, when the EC max exceeds 95 °C (abort and restore the floors), when the argmax label changes mid-step, or when the fit is rejected. `RunnerEffect::SaveState` now carries `loop_gains`; `progress().phase` reports `"lut"`/`"step"` as plain strings. The controller's one-line call-site change passes `CalibContext::default()` and ignores `SetBudget` until fwloop.19 wires them. **owns:** the `StepTest` phase incl. burner/`NeedsLoad` handling, `CalibContext` struct, `RunnerEffect::SetBudget`, the `fitted_at` stamp, the phase label strings. **consumes:** `LoopGains` (fwloop.4), `PersistedState.loop_gains` (fwloop.11), `fit_fopdt`/`derive_gains` (fwloop.21). blocked-by fwloop.4: consumes `LoopGains`. blocked-by fwloop.11: consumes the `PersistedState.loop_gains` slot. blocked-by fwloop.21: consumes `fit_fopdt` + `derive_gains`. Files: `src/calib/runner.rs`, `src/calib/step.rs`, `src/calib/mod.rs`, `src/control/controller.rs` (call site only). Acceptance: runner end-to-end test on the fake seams walks sweep → burner → settle → step → `SaveState` with gains carrying `fitted_at`, driven by scripted `CalibContext`s; the burner starts before the settle hold and stops after; a scripted run whose idle argmax is uncontrollable but whose loaded argmax is controllable **proceeds** rather than self-skipping; an unloaded step, a 95 °C abort, a mid-step argmax change and a rejected fit each skip with a `Noted` reason and keep defaults; the gain is computed from the measured per-axis delta, not 30 W; no `fit_batch`/`MATRIX_POINTS` symbol remains in `calib/`. | PROMOTE overruled → LEAF (demoted-by-session) · [cov-1] · [cov-2] |
| fwloop.14 | 1 | fwloop.5, fwloop.13, fwloop.15, fwloop.22 | Deletion sweep | Delete `src/control/thermal_model.rs`, `kalman.rs`, `trust.rs`, `cooldown.rs`, `trim.rs`, their `mod.rs` lines, and every remaining reference (the controller's tier and tests go with fwloop.22, the `state.rs` fit tests with fwloop.11, the matrix code with fwloop.13, the telemetry fields with fwloop.15 — this task removes whatever is left, e.g. `gpu_pid.rs` comments, stray imports) plus `TODO.md`/docs mentions. Search the repo for each path and basename in source and non-source files (docs, TODO.md, README, any manifest) — every hit is removed here or owned by fwloop.18 (README). blocked-by fwloop.5: consumes the allocator rewrite that removes the last `contour`/`CONSERVATIVE_START`/`overshoot_settle` references and orphans `trim.rs` (this task deletes the file). blocked-by fwloop.22: consumes the tier-free controller (no importer left in `controller.rs`). blocked-by fwloop.13: consumes the matrix-free runner (no `fit_batch` caller). blocked-by fwloop.15: consumes the telemetry field removal (`trim_rpm`/`gain`/`model_*`). Files: the five modules, `src/control/mod.rs`, `src/control/gpu_pid.rs`, `TODO.md`. Acceptance: a repo-wide search for `thermal_model`, `kalman`, `trust::`, `cooldown`, `trim::`, `adapt_bias`, `ModelSnapshot`, `trim_rpm`, `contour`, `CONSERVATIVE_START`, `overshoot_settle` hits only `docs/research/`, `docs/plans/` history, this spec, and `README.md` (owned by fwloop.18); no `src/` hit remains; `cargo test` and `cargo clippy -D warnings` green. | [cov-1] · [cov-2] re-pointed 12→22 |
| fwloop.15 | 1 | fwloop.8, fwloop.9 | TUI + telemetry surface | `src/ui/view.rs`: header segment `mode A|B|rel · T* · ma · duty → rpm · budget`; render and rank the new flags incl. `READBACK BLIND` (info); calibration progress renders `CalibProgressLite.phase` as the plain string it already is. `src/telemetry.rs`: sample fields `ec_max`, `ec_argmax`, `ec_ma`, `nvme_c`, `fanctrl_speed`, `fanctrl_active`, `strategy`; decision fields `mode`, `t_star`, `budget_w`, `freeze`; remove `trim_rpm`, `gain`, `model_*`. **consumes:** `ControlStatus` fields + flags (fwloop.8); `Sample.ec/nvme_temp_c/fanctrl` (fwloop.9). blocked-by fwloop.8: consumes the `ControlStatus`/`StatusFlag` surface. blocked-by fwloop.9: consumes the new `Sample` fields serialised into telemetry. Files: `src/ui/view.rs`, `src/telemetry.rs`, `src/model.rs`. Acceptance: view snapshot tests for each mode and each new flag; a telemetry line serialises the new fields and omits the removed ones. | [cov-1] · [cov-2] |
| fwloop.16 | 1 | fwloop.1, fwloop.2, fwloop.3, fwloop.9, fwloop.20 | fw-fanctrl emulator + chained plant | `src/test_support/plant.rs` (cfg(test)): `FanctrlEmulator` (1 s tick, boxcar N non-zero with the off-by-one, `eff=min(MA,cur)`, `Curve`, `int()` truncation, `active` switchable, `edit_curve_in_place(points)` under the unchanged strategy name; **two upstream quirks reproduced**: no history append while paused, and a hardcoded 50 °C injected on a scripted sensor-read failure — these are what make the replica's average diverge from the socket's in ways the instantaneous value hides; `view(now) -> FanctrlView` with curve, `ma_temperature`, `ma_interval`, `active`, both stamps, plus a `Freshness` injection hook for socket death that **also switches the plant into EC-autofan mode**, since a stopped fw-fanctrl leaves the fans on the EC curve via its unit's `ExecStopPost --autofanctrl`, making `absent` and `active: false` one plant regime, §2.5), `ThermalPlant` (watts → controllable EC °C, τ 35, θ 20, K 0.8, plus separately labelled `ambient`/`charger` channels and scriptable `gpu_*` channels emitted as an `EcReading`), `FanPlant` (duty → RPM via **its own table**, seeded from the same points with a configurable per-duty offset; one-sided momentum kick on positive slew; ±90 RPM noise from a hand-rolled seeded xorshift, no new dependency) plus **an EC-autofan mode** used whenever the emulator is `active: false`, driving RPM straight from EC temperature with the measured staircase of §Facts (4096 RPM at 61 °C, 4520 at 63, 4658 at 64, flat 4748 across 67–73) — the 67–73 plateau is well sampled and load-bearing for the no-authority result, while the steep 61–64 segment is thin and its rising branch and hysteresis are unmeasured (§Facts limitation), so treat that part as approximate and do not derive claims about the EC from it, scriptable `gpu_temp_c`/`nvme_temp_c` (both `Option`) and `cpu_util`/`gpu_util`/`gpu_sm_mhz` tracks, **a demand model that decides how much of the commanded cap is actually drawn** (so `cpu_pkg_w`/`gpu_w` on the emitted `Sample` are a measured draw that can sit well below the cap — without this the plant cannot exercise the demand-starved wind-up at all), a scriptable `resumed` edge, `ChainedPlant` composing them and producing a full `Sample` per tick (`fanctrl_view_changed` set once per new `print all` view). **owns:** the plant API and its RNG. **consumes:** `Curve` (fwloop.1), `FanctrlView`/`Freshness` (fwloop.2), `EcReading`/`EcLabel` (fwloop.3), the `Sample` field set (fwloop.9), the curve fixtures and the `plant.rs` slot (fwloop.20). blocked-by fwloop.1: consumes `Curve::duty_at`. blocked-by fwloop.2: consumes `FanctrlView` (two stamps) + `Freshness`. blocked-by fwloop.3: consumes `EcReading`/`EcLabel`. blocked-by fwloop.9: consumes the extended `Sample` shape incl. `fanctrl_view_changed`, `fanctrl_freshness`, `ec_valid`, `on_ac`. blocked-by fwloop.20: consumes the `quiet16`/`cool16` fixtures and `src/test_support/plant.rs`. Files: `src/test_support/plant.rs`. Acceptance: emulator reproduces the verified truncation case (T_eff 51.8 → 21 on cool16) and the off-by-one; a scripted temperature drop drives `eff` from the `current` branch; a scripted socket death yields `Absent` **and puts the fan plant on the measured EC staircase**; an in-place curve edit yields new points under the same name in the emitted view; `fanctrl_view_changed` is set exactly once per new `print all` view; an open-loop step on the chained plant shows a 26–30 s watts→RPM lag; the plant table offset shifts steady RPM by the configured amount; scripted CPU-heavy vs GPU-heavy utilisation shifts the demand split; a low-demand script emits a measured draw well below the commanded cap; a paused emulator stops appending history, a scripted read failure injects 50 °C, and an `active: false` emulator drives RPM from the measured EC staircase instead of the commanded duty. | [cov-1] · [cov-2] |
| fwloop.17 | 1 | fwloop.12, fwloop.16, fwloop.19 | Closed-loop acceptance + configuration smoke | Controller-level sims on `ChainedPlant` through the real `on_sample`, with fwloop.19's hooks active, the load step being a utilisation + watts step: for each of `quiet16`, `cool16` × TempLoop, RpmLoop (4 runs): load step then 30 min — ≥ 90 % inside ±150 RPM, **no relay under the period-agnostic rule of §5** (no ≥ 3 consecutive sign-alternating band excursions at any period; the count and dominant period are reported); the same 4 runs with plant K/τ/θ perturbed ±50 %; a run with the plant table biased −8 % where refinement brings RPM inside ±150 within 20 min and T* follows the re-snapped duty; **a demand-starved run** (long idle with the plant drawing far below the cap, then a load onset) where `u` never reaches the upper bound and the onset overshoot stays inside ±150; a calibration run (StepTest on the plant, then the quiet16/TempLoop acceptance repeated with the fitted gains meeting the same bar) — **the fit is graded against the boxcar-filtered plant it actually sees, not the raw τ 35 / θ 20 constants**, so the criterion is that the derived `Kc` lands within 25 % of the IMC value for that filtered plant and that the closed loop passes, never that τ/θ match the raw plant; a load-release transient at t=1200 that stays inside ±150 within 90 s; a dGPU-powered-and-hot 30 min run that stays in TempLoop with no `EC MISMATCH`; a dGPU-unpowered run (no `GPU HOT`, floor honoured, `verify_lock` `Unverifiable`); a sub-floor target run that raises `TARGET UNREACHABLE (low)` and holds the floor without relay. Bumpless: socket death at t=600 (A→B), `active:false` at t=700 with a fresh socket (A→B), and an in-place same-name curve edit at t=900 leave |Δu| ≤ one increment and the caps continuous. **`active: false` authority run** (plant in EC-autofan mode, §Facts staircase): with a target below the EC's flat band the loop parks `u` at the floor, raises `TARGET UNREACHABLE (low)` naming the achievable RPM within 60 s, and the integrator does not wind; with the EC on its steep segment below 64 °C the loop does not hunt. **The same run repeated with the socket `Absent`** (the killed-daemon regime, §2.5) behaves identically and additionally raises `FANCTRL LOST`. **Demand-limited runs:** a duty-cycled load (5 min on, 2 min off, ×3) must not let `u` decay toward the lull draw and must return inside ±150 RPM within 90 s of each onset; a CPU-only run with the dGPU unpowered must leave `u` off its lower bound and `cpu_w` above `cpu_floor_w` after 10 min. **Rejected-curve run:** a non-monotone curve keeps the loop in RpmLoop at the `0.25×` gain clamp with `CURVE INVALID` raised, and never `SteepCurve`. `Released`: socket absent **and** invalid fan reading ⇒ stock caps within one hysteresis window, `FANCTRL LOST` + `SENSOR LOST` set, then sensor recovery re-engages RpmLoop from the warm-start without a cap step. Faults: feasibility (T* < ambient + 5), the `high` unreachable case (target above the fans' reach), steep-curve flag, single read-back mismatch freeze, a mismatch suppressed across an `on_ac` edge, three-strike release followed by a `Verified` re-engagement, a 5 min `GPU HOT` episode at the 90 °C threshold with no post-episode overshoot > 150 RPM, an `NVME HOT` episode that raises the flag while the RPM trace is indistinguishable from the same run without it, reconciliation A→B→A with reseed, a scored view skipped because the replica was slewing, and a `resumed` edge mid-run that clears the windows and writes no warm-start or refinement across the gap. Global assertions over every run: only `Speed`/`All` commands were ever recorded by the fake; `ec_ma_c` tracks the emulator's `ma_temperature` within 1 °C in steady state; at least one steady window is detected per converged run. **consumes:** the `on_sample` data flow (fwloop.12), the warm-start/refinement/calibration hooks (fwloop.19), `ChainedPlant` (fwloop.16). blocked-by fwloop.12: consumes the integrated auto loop. blocked-by fwloop.16: consumes `ChainedPlant`. blocked-by fwloop.19: consumes the warm-start/refinement/calibration hooks so the runs grade the final loop. Files: `src/control/sim_tests.rs` (cfg(test)), `src/control/mod.rs`. Acceptance: all listed runs pass deterministically (seeded RNG); each spec-enumerated configuration (2 strategies × 3 modes, dGPU on/off, default vs fitted gains) is exercised end to end (needs: fwloop.12, needs: fwloop.19). | [cov-1] · [cov-2] |
| fwloop.18 | 1 | fwloop.2, fwloop.6, fwloop.8, fwloop.13 | README + docs | README: rewrite Calibration walkthrough (LUT sweep + step test, burner, `NeedsLoad`), Auto mode (cascade, modes, flags table), Safety model (fw-fanctrl owns the fans, socket read-only by construction, guards, read-back incl. `READBACK BLIND`), Configuration table (drop `online_rls`, add `fanctrl_socket`, `gpu_hot_c`, `nvme_hot_c`), and a Safety-model line saying plainly that the NVMe reading is reported and not acted on, with the measurement behind that (§2.8); `docs/research/03-control.md` pointer note; INDEX status → implemented. **consumes:** config key names (fwloop.2, fwloop.6), the `StatusFlag`/`LoopMode` set (fwloop.8), calibration flow (fwloop.13). blocked-by fwloop.2: consumes the `fanctrl_socket` key name/default. blocked-by fwloop.6: consumes the guard key names/defaults. blocked-by fwloop.8: consumes the flag and mode names for the flags table. blocked-by fwloop.13: consumes the step-test flow (durations, skip reasons). Files: `README.md`, `docs/research/03-control.md`, `docs/superpowers/specs/INDEX.md`. Acceptance: every config key in `config.rs` appears in the README table and vice versa; every `StatusFlag` variant appears in the flags table; no README mention of matrix/model/trim/RLS remains. | [cov-1] |
| fwloop.19 | 1 | fwloop.12, fwloop.13 | Controller hooks: warm-start, refinement, calibration | `src/control/controller.rs`: the steady-window detector on the **`rpm_smoothed`** series (population stdev < 60 over 40 s, `active`, the view's own `speed_pct` equal to `target_duty` for the whole window, no guard override, `u` off both bounds — §2.3, so a window whose achieved duty differs from the target's never writes into the target's entry) that (a) records `warm_start[key(strategy, duty, on_ac)] = u` and (b) calls `DutyRpmTable::refine(duty, mean_rpm)`, and when the snapped duty changes surfaces the new `target_duty` to the arbiter (T* derivation stays in fwloop.10; the controller calls `resync_error` on `t_star_changed`); warm-start seeding of `u` **only** on auto entry, on re-engagement from `Released`, and at calibration exit (fallback: the floors) — a mid-session key change re-keys the recording target and never re-seeds (§2.4); building `CalibContext` each sample from the arbiter's decision and the budget bounds; asserting `Freeze::Calibrating` for the whole calibration session (LUT sweep included) and applying `RunnerEffect::SetBudget` (seed `u = w`, normal split + command path); persisting the table, warm-start map and `loop_gains` through `save_persisted_state`. **owns:** the steady-window detector, warm-start seed/record and the no-reseed-on-key-change rule, refinement trigger, the whole-session calibration freeze, the `CalibContext`/`SetBudget` ends on the controller side. **consumes:** the wired auto loop incl. budget bounds and `resync_error` (fwloop.12), `CalibContext`/`SetBudget` (fwloop.13), `WarmStart` (fwloop.4, transitively through 12), `DutyRpmTable::refine` (fwloop.1, transitively). blocked-by fwloop.12: consumes the integrated `on_auto_sample`, `EcAverage` instance and budget bounds. blocked-by fwloop.13: consumes `CalibContext` and `RunnerEffect::SetBudget`. Files: `src/control/controller.rs`. Acceptance: a steady window on the smoothed series records both the warm-start entry and a table refinement, and a window on raw ±90 RPM noise still qualifies; auto entry seeds `u` from a matching key, floors otherwise; a strategy change, a snapped-duty change and an AC unplug each re-key without re-seeding (that tick's Δu equals the ordinary PI increment); re-engagement from `Released` seeds from the warm-start; the integrator is frozen from calibration start through exit and `u` is unchanged across a LUT sweep; a `SetBudget` effect lands the requested budget through `split_budget` and commands it; `CalibContext` mirrors the arbiter's decision and the budget bounds; the persisted file round-trips all three. | [cov-1] · [cov-2] |
| fwloop.23 | 1 | fwloop.1, fwloop.2, fwloop.3, fwloop.4, fwloop.5, fwloop.6, fwloop.7, fwloop.8, fwloop.9, fwloop.10, fwloop.11, fwloop.12, fwloop.13, fwloop.14, fwloop.15, fwloop.16, fwloop.17, fwloop.18, fwloop.19, fwloop.20, fwloop.21, fwloop.22, fwloop.24 | Integration sweep: fw-fanctrl closed loop | Verify the goal's main flows end to end on the merged tree and implement what is missing: (1) engage auto from a fresh `state.json` with only a LUT, walk TempLoop → socket death → RpmLoop → recovery → TempLoop, run a calibration, restart the daemon and confirm the warm-start, table and gains reload; (2) sweep for unwired config values, parameters and interfaces — every `Config` key is read somewhere, every `StatusFlag` is raised somewhere and rendered, every telemetry field is populated, every `Effect` variant is applied, `CalibContext` fields all originate from live data; (3) add the integration tests no per-task test covers (sampler → controller → telemetry line with real types; config → poller construction; a full `on_command`/`on_sample` session on the fakes); fix small gaps inline, file blockers for large ones. blocked-by fwloop.1: consumes all leaves (integration sweep). blocked-by fwloop.2: consumes all leaves (integration sweep). blocked-by fwloop.3: consumes all leaves (integration sweep). blocked-by fwloop.4: consumes all leaves (integration sweep). blocked-by fwloop.5: consumes all leaves (integration sweep). blocked-by fwloop.6: consumes all leaves (integration sweep). blocked-by fwloop.7: consumes all leaves (integration sweep). blocked-by fwloop.8: consumes all leaves (integration sweep). blocked-by fwloop.9: consumes all leaves (integration sweep). blocked-by fwloop.10: consumes all leaves (integration sweep). blocked-by fwloop.11: consumes all leaves (integration sweep). blocked-by fwloop.12: consumes all leaves (integration sweep). blocked-by fwloop.13: consumes all leaves (integration sweep). blocked-by fwloop.14: consumes all leaves (integration sweep). blocked-by fwloop.15: consumes all leaves (integration sweep). blocked-by fwloop.16: consumes all leaves (integration sweep). blocked-by fwloop.17: consumes all leaves (integration sweep). blocked-by fwloop.18: consumes all leaves (integration sweep). blocked-by fwloop.19: consumes all leaves (integration sweep). blocked-by fwloop.20: consumes all leaves (integration sweep). blocked-by fwloop.21: consumes all leaves (integration sweep). blocked-by fwloop.22: consumes all leaves (integration sweep). blocked-by fwloop.24: consumes all leaves (integration sweep). Files: `src/**` (small inline fixes), `src/integration_tests.rs` (cfg(test)). Acceptance: the three flows above pass on the fakes; the unwired-sweep checklist is recorded in the spec's Post-Implementation Notes with zero open items or a filed blocker per item; `cargo test` and `cargo clippy -D warnings` green. | root integration sweep |

Graph shape: longest chain fwloop.20 → 2/3 → 9 → 12 → 19 → 17 → 23 (7 rounds); fwloop.24 (the anti-windup spike) hangs off fwloop.4 and runs abreast of the sensor work, so it adds no round despite gating the hub. Round 1
readies fwloop.20, 1, 4, 5, 6, 8; fwloop.22 (tier removal) now lands in round 2 abreast of the
sensor work, so fwloop.14 can run as soon as 13 and 15 have landed instead of waiting for the
hub. fwloop.12 is pure wiring; fwloop.19 carries the separable hooks; fwloop.17 waits for 19 so
the acceptance grades the shipping loop; fwloop.23 is the terminal join.

## Post-Implementation Notes

*As this design is implemented and iterated on — bug fixes, adjustments, anything that diverged from the assumptions above — append a dated note here, whether or not a formal debugging skill was used.*

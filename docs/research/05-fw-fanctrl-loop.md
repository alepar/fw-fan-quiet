# 05 — Closing the loop on fw-fanctrl instead of learning power→RPM

Research notes, 2026-09-07. Input for a `super-design` run on the refactor. Everything below was
verified on this machine (bazerame) unless marked *reported*. Motivation for the refactor is
**robustness**, not loop tightness — the analysis concludes the loop should be *slow*.

## 1. What fw-fanctrl actually is (v1.1.0, `fw-fanctrl-0.0.0-11.20260606.gitb040da6`)

Source: `/usr/lib/python3.14/site-packages/fw_fanctrl/`. Only backend is `framework_tool`
(`ectool` backend removed; `--no-battery-sensors` gone).

- **Sensor = MAX over every line of `framework_tool --thermal`** matching `:\s*(\d+)\sC`, zeros
  dropped. Battery is included. `Charger IC` is silently excluded (label has no colon).
  On any `framework_tool` failure it returns a **hardcoded 50 °C**.
- **NOT k10temp/Tctl.** bazerame-fans reads `k10temp/temp1_input` — a different signal.
  Measured 2026-09-07 at idle: `ambient_f75303@4d`=48 °C was the argmax while `cpu@4c`=39 °C;
  fw-fanctrl reported `temperature: 48.0`. Under CPU load earlier the same day `cpu@4c`=78.8 °C
  was the argmax while ambient=61.9 °C. **The argmax sensor switches by regime.**
- Effective temperature: `min(moving_average, current)` — `FanController.py:138-140`. The
  `# 2/3 of the effective temperature` comment is stale; there is no weighting. Consequence:
  heavily damped on *rising* temps, **instant** on *falling* temps. This is undocumented
  upstream (docs say only "moving average") and may change in any release.
- Moving average: boxcar over the last N *non-zero samples* (not N seconds), N = `movingAverageInterval`
  capped at 100 (deque maxlen). Sampled ~once per tick.
- **Off-by-one:** `adapt_speed` runs *before* the tick's sample is appended, so duty at tick n uses
  mean(samples n-N…n-1) but `min` against sample n.
- Interpolation: piecewise linear in file order (never sorted), `int()` **truncation** (verified:
  T_eff=51.8 → 21 not 22). Flat clamp below first / above last point. No hysteresis, no rate
  limit, no dead band, no dirty check — `framework_tool --fansetduty <int>` is written every
  update tick unconditionally.
- Tick is `sleep(1)` + ~3 subprocess forks (`--thermal`, `--power` ×2) ≈ **1.02–1.06 s, free-running**.
- `pause` → `framework_tool --autofanctrl` (EC takes over with its own unknown curve). `resume` is a
  hardware no-op; control returns at the next `--fansetduty`. Buffer survives pause/strategy change,
  so the first post-resume duty averages stale samples.
- Daemon exits(1) on any exception; unit is `Restart=always`; `ExecStopPost` runs `--autofanctrl`.

### The socket — the robust way to know the live curve
`/run/fw-fanctrl/.fw-fanctrl.commands.sock`, AF_UNIX stream, **mode 0777** (no root). Protocol:
send the raw CLI arg string, read to EOF, one command per connection, no framing/auth.
`--output-format JSON print all` returns fw-fanctrl's in-memory state: resolved `strategy`, `speed`
(last *commanded* duty), `temperature`, `movingAverageTemperature`, `effectiveTemperature`,
`active`, and `configuration.data` (the entire parsed config). `print speed` is cheap (no fork);
`print all` forks `framework_tool` twice per call — not for a 1 Hz loop.
**The config file is not authoritative:** `Configuration.reload()` runs only at init and on an
explicit `reload`; `set_config` (any local user) rewrites the file as root. Trust the socket.
(0777 + `set_config`-as-root is an upstream local-privilege smell; note, don't rely on it.)

## 2. Measured on this machine

Duty→RPM (steady, fans paused/manual, fan0≈fan1): 15%≈1195, 20%≈1670, 27%≈2300, 30%≈2560,
36%≈3030, 40%≈3380, 44%≈3670, 48%≈3950, 52%≈4180, 85%≈5920. ~3500 RPM ≈ 42% duty.
Readable without root: EC temps `/sys/class/hwmon/hwmon10/temp*_{label,input}` (`cros_ec`),
fan RPM `/sys/class/hwmon/hwmon7/fan{1,2}_input` (`framework_laptop`). hwmon numbers may renumber.
Idle EC readings 2026-09-07: ambient 48, charger 44, apu 42, cpu@4c 39; all four `gpu_*` EC
sensors read 0/ENODATA — **the dGPU does not feed the fan curve directly**, only by soak into
apu/ambient/charger with extra lag. RTX 5070 has its own internal 87 °C target loop.
Active strategies (config as of 2026-09-07): `quiet16` (15%→55 °C, 21@65, 31@75, 37@82, 55@88,
100@95; freq 1, avg 60) and `cool16` (20%→50, 30@60, 42@70, 100@85; freq 1, avg 60).
Slopes: quiet16 65–82 °C ≈ 0.9–1.0 %/°C (benign); cool16 70–85 ≈ 3.9 %/°C (hunting-prone, >2 %/°C).
freq=1 + avg=60 measured: 1-point duty steps (~55 RPM) on ramp-up; 1.45% of one core.

### 2026-09-09 — GPU load alone raises fan RPM (the dGPU DOES feed the EC curve)

An earlier entry in the design doc's §Facts claimed the cros_ec `gpu_*` sensors "read −150" and
that the dGPU does not feed the EC fan curve. Re-measured; **false on this machine.** The "−150"
was the EC's unset-*threshold* sentinel (`sensors` shows `high/crit/emerg = −273.1 °C` on
`gpu_amb`/`gpu_vram`), not a reading — all four `gpu_*` inputs are live through `ectool temps
all`, the path fw-fanctrl itself reads, and `gpu_temp@40` tracks NVML's die temperature within
1 °C. The EC also carries real thresholds on them: `gpu_vr` high **70.8** / crit 89.8 / emerg
104.8; `gpu_temp@40` high 86.8 / crit 96.8 / emerg 99.8.

Two 240 s gpu-burn runs (`localhost/gpu_burn:blackwell`, ~100 W, CPU idle), sampled every 2 s
(`2026-09-09-gpu-burn-fanctrl-quiet16.csv`, `2026-09-09-gpu-burn-ec-native.csv`):

| | **A: fw-fanctrl `quiet16`** | **B: EC native (`ectool autofanctrl`)** |
|---|---|---|
| pre-load argmax | `ambient` 61 °C | `ambient` 58 °C |
| first GPU-sensor argmax | **t = 13 s** (`gpu_temp@40` 67) | **t = 14 s** |
| steady-state argmax | `gpu_vr` | `gpu_vr` |
| GPU die, steady mean / max | 82.4 / 83 (peak 85 on the ramp) | 76.0 / 77 |
| `gpu_vr`, steady mean / max | 83.2 / 84 | 76.6 / 78 |
| `ambient`, steady | 64.5 (from 61) | 61.9 |
| `cpu@4c`, steady | 47 (**fell** from 53) | 44 |
| fan0, steady mean / max | **3330 / 3461 rpm** (from 1478, duty 18 → 38 %) | **4002 / 4028 rpm** |
| 40 s after load-off | `gpu_vr` 71, die 53, fan 2313 | `gpu_vr` 67, die 50, fan 2368 |

What this establishes:
- **A GPU sensor takes the argmax ~13 s into GPU-only load and never gives it back.** The fans
  went 1478 → 3330 rpm with the CPU idle and `ambient` moving only 61 → 65: the ramp was driven
  by `gpu_temp@40` first and `gpu_vr` at steady state, not by any CPU/chassis sensor.
- **`gpu_vr` runs ~1 °C above the die and has a long tail** — still 67–71 °C forty seconds after
  the die is back at 50. A guard keyed on the EC argmax holds longer than the die suggests.
- **The card does not reach its 87 °C park point with the fans free.** It settles at 82–83 on
  `quiet16` and 76 under the EC's native (~670 rpm more aggressive) curve. 87 is reached only
  when something holds the fans below what the GPU sensors ask for — which is what this
  controller does for a quiet RPM target. So a GPU reading at/above 87 in auto mode is a genuine
  "our cap is the limiting factor" signal, not normal operation.
- **Consequence for the design:** "release caps to stock" on a GPU thermal trip *does* summon
  cooling (the curve sees the GPU), so a hard trip is coherent; and the dGPU guard needs no
  separate NVML-only path to be reachable by the fans (open question 3 below, now resolved).
  NVML T.Limit specs on this card: park/max-operating 87, **Slowdown 89**, **Shutdown 92** — a
  5 °C band, which is why the guard/trip pair became 88 / 91 with a 2 °C hysteresis
  (fw-fanctrl-loop-a78).

## 3. bazerame-fans code map (2026-09-07, 19,452 lines incl. tests)

Control loop: `control/controller.rs` (`on_auto_sample` ~:985-1329); allocator 5 s
(`ALLOC_PERIOD_S`); GPU PI 1 Hz (`control/gpu_pid.rs`); Kalman 20 s. Fan RPM is the controlled
variable (`Sample::max_fan_rpm()`); `Sample.cpu_temp_c` (Tctl) is plumbed with a validity flag but
sits in **no control path** (watchdog + UI only).

**Becomes deletable if the loop closes on fw-fanctrl's state:** `control/thermal_model.rs` (783),
`control/kalman.rs` (572), `control/trust.rs` (165), `control/cooldown.rs` (166) entirely; the
matrix phase of `calib/runner.rs` (~half of 1137: `MATRIX_POINTS`, `Phase::MatrixPoint`,
`fit_batch` call); the controller's five-gate adaptation tier (`controller.rs:1176-1315`);
`PersistedState.adapt_bias/adapt_gain`; `ControlStatus.trim_rpm/gain`; `Effect::ModelSnapshot`.
Production code ≈1,000 lines; the rest is tests that go with it.

**Must change:** `AllocInput.contour` (`allocator.rs:249`) — a temperature/duty loop yields a
*scalar budget*, not a (pc,pg) contour; `best_candidate` becomes "split a budget"; every band
constant is in RPM (`DEADBAND_RPM=150`, `RAISE_HOLD_RPM=50`, `SLOPE_GATE_RPM_S=10`) and needs
re-derivation; timing constants tuned to the 26–30 s watts→RPM lag can become *derived* from
`movingAverageInterval` read off the socket; `state.json` drops `model`, keeps `lut`; Auto-entry
precondition (`controller.rs:698`) changes; README safety narrative changes.

**Survives unchanged:** all of `sensors/`, `actuators/`, `guard.rs`, `gpu_pid.rs`+`lut.rs`+
`lut_sweep.rs` (the GPU watts→clock inner loop is orthogonal), `watchdog.rs`, `telemetry.rs`,
`led/`, `allocator::demand` (starvation scoring).

Note: `docs/research/03-control.md` already described fw-fanctrl as "a pure temperature→RPM curve
driver"; the 5 s allocator cadence was derived from it. Live `state.json` model
(`a=110.893 b=26.908 e=-0.46594 c=-18.234`) matches the numbers cited in `thermal_model.rs:33-41`
for the June degenerate-divisor incident — confirm it isn't the degenerate one.

## 4. Control analysis (researched; the load-bearing claims)

- **Setpoint self-consistency holds.** fw-fanctrl is a static, memoryless, monotone function of
  its input; at equilibrium MA=current. Regulate its *input* and its *output* follows by
  construction, independent of the thermal model. The thermal model only says *what wattage* is
  achievable at T* — which closed-loop feedback finds without a model.
  Requirements: steady state; regulate the **same signal** fw-fanctrl uses (max EC sensor, not
  Tctl); the argmax must be one you can influence; fw-fanctrl running and unpaused.
- **Control the moving average, not instantaneous temp.** Then `eff=min(MA,cur) ≤ MA` and duty
  never exceeds curve(MA). Caveat (dissent, §6): this bounds noise only *to the extent the
  controller keeps MA ≤ T\**; a fast load step can still push MA past T* faster than a slow PI
  reacts. The genuine transient guarantee — fans follow MA, so a spike can't spike the fans — is
  a property of fw-fanctrl's boxcar and holds in *every* design, RPM-feedback included.
- **Concavity bias:** `min` is concave ⇒ temperature variance biases delivered RPM *downward*.
  Oscillation costs watts, never noise. Argues for sluggish, well-damped tuning; failure mode is
  under-delivered power.
- **Two controllers, one measurement is benign here** because the combined loop has exactly one
  integrator (fw-fanctrl is P-only). The fan loop *adds* negative feedback (k_eff > k, pole moves
  left). Risk: the fan loop's own delayed feedback (~N/2 s) can hunt on its own on steep segments —
  refuse/warn on targets landing on curve slopes > ~2 %/°C.
- **Never run two PIs (CPU cap, GPU cap) on one sensor** — non-identifiable; the integrators
  random-walk until one saturates. Correct structure: **one integrator → scalar budget → static
  allocation** (user preference: GPU-priority / CPU-priority / ratio). This is control
  *allocation* (2-in-1-out), not MIMO; nothing to decouple.
- **Gain scheduling:** plant gain dT/dP ∝ 1/k(n), k ∝ n^0.8 ⇒ 2–3× variation across duty range,
  **worst at low RPM** (where we operate). Integral action fixes offset, *not* stability margin.
  Schedule K_c on *commanded duty* (exact, lag-free), power-law 0.8, clamped [0.5,2]×; do it after
  the fixed-gain loop works.
- **Recommended cascade:** slow RPM trim (λ 300–600 s) adjusting T\* ← PI on MA(max EC) (λ≈90 s,
  velocity form, **no D** — D amplifies boxcar edge artifacts) → fw-fanctrl. The RPM trim absorbs
  ambient, dust, fan ageing, curve edits with no model. Tune with lambda/IMC, not Ziegler–Nichols
  (ZN targets oscillation, which concavity penalizes). Ballpark τ≈35 s, θ≈20 s, K≈0.8 °C/W ⇒
  K_c≈0.4 W/°C, T_i≈35 s at λ=90 — **run a step test with fw-fanctrl running**, never with fans
  pinned. Relay autotune is biased by the `min` asymmetry; prefer the step test.
- **Anti-windup:** clamping + back-calculation; freeze/unwind also on loss of controllability
  (argmax is ambient/charger), actuator write-back mismatch, fw-fanctrl paused/dead/mid-switch.
  Bumpless re-init on every argmax-sensor switch.
- **Feasibility gate:** T\* must exceed max(uncontrollable sensors)+~5 °C or the target is
  physically unreachable — report it, don't wind down to the floor for nothing.
- **Setpoint deadband for free:** duty is int-truncated, so the inverse curve is a staircase;
  set T\* at the centre of the target duty's tread (±0.5 °C of zero duty change) and report the
  achievable RPM grid rather than accepting arbitrary targets.
- **Prior art:** thermald ships P-only (`ki=0`) and its escalation fallback produces 6–11 °C bang-
  bang limit cycles (issue #146) — avoid; fan2go PID defaults p=0.3 i=0.02 d=0.005; RyzenAdj
  `--tctl-temp` is a hardware temperature loop worth considering for the CPU leg.

## 5. Actuator caveats (reported, verify)

- `ryzenadj` on Ryzen AI 9 HX 370 **silently fails** to write `slow-time`/`stapm-time`
  (FlyGoat/RyzenAdj #412). Read back every write; treat mismatch as actuator failure (else the
  integrator winds up against an actuator that never moved).
- Use `--slow-limit` as the handle, not `--stapm-limit` (STAPM is itself a long-window average —
  a second slow filter in series). `--fast-limit` bounds excursions.
- AMD `intel-rapl:0` has `energy_uj` only, no `constraint_*_power_limit_uw` — RAPL is read-only here.
- `nvidia-smi --query-gpu=power.limit` returns N/A on driver 610.x; use NVML and read back
  `enforced.power.limit` (min of all limiters). `nvidia-powerd` is inactive; it would fight a cap.

## 6. Design options and a dissent to weigh in brainstorming

- **Mode A (researchers' primary):** PI on MA(max EC sensor), setpoint from inverting the live
  curve read off the socket. Needs: sensor-set replication or socket polling for the temps,
  argmax-switch detection, bumpless transfer on strategy change, feasibility gate, and it rests on
  the undocumented `min()`.
- **Mode B (researchers' fallback):** PI on **measured RPM** directly (or on commanded duty read
  via `print speed`). Same integrator, same actuator, same anti-windup; only the controlled
  variable and setpoint change. Needs nothing from fw-fanctrl internals.
- **Dissent (session's own view):** build Mode B first. It captures the deletion, the robustness,
  and the boxcar's transient smoothing; Mode A adds seconds of loop latency against a λ=90 s loop,
  plus most of the complexity and a dependency on a detail that changes silently (fw-fanctrl
  already self-updated 1.0.2→1.1.0 unnoticed). Add curve inversion only if Mode B measurably
  undershoots/lags. Either way: poll `print all` occasionally for the live strategy/curve/`active`,
  fall back between modes bumplessly, and demote the persisted Kalman state to an **integrator
  warm-start** keyed by (strategy, target, AC, ambient bucket).

## 7. Open questions for the design
1. User-facing unit: RPM (needs the measured duty→RPM table), duty %, or temperature?
2. CPU/GPU allocation policy exposed how (fixed ratio / priority / measured-share)?
3. Independent NVML GPU-temperature guard outside the noise loop — required (dGPU invisible to EC).
   **Premise refuted 2026-09-09** (§2 above): the dGPU is visible to the EC and dominates the
   curve under GPU load. The guard remains useful as the *ratchet-down* response, but not because
   the fans can't see the card.
4. NVMe (SN850X, 90 °C warn, 66 min cumulative above it) is invisible to fw-fanctrl too — in
   scope as a guard, or out of scope?
5. What happens on `active:false` / socket missing: hold last cap, release to stock, or RPM mode?

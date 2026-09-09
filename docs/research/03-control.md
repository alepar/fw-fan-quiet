> **2026-09-09 note:** the learned power→RPM thermal model this document designs around
> (Section 2's affine-plus-cross-term / softplus model, Section 2's RLS/EWMA online
> adaptation, and the Section 5 calibration matrix that fit it) has been replaced. The
> shipped design instead closes the loop through fw-fanctrl's own temperature→RPM curve
> with a single PI integrator, has no learned thermal model, and calibrates with a GPU
> clock→watts sweep plus a step test rather than a full (CPU W × GPU W) matrix. See
> [`docs/research/05-fw-fanctrl-loop.md`](05-fw-fanctrl-loop.md) for the verified research
> behind that design and
> [`docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md`](../superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md)
> for the design itself. This document remains useful as prior-art/background research
> (actuator behavior, time constants, demand estimation) but its Section 2 model form and
> Section 5 calibration matrix no longer describe the shipped system.

# Designing a Noise-Targeted Power-Shaping Thermal Controller for the Framework 16 (Ryzen AI HX 370 + RTX 5070 Mobile, Linux)

## TL;DR
- **The architecture is sound and novel**: no existing Linux tool models the (CPU_W, GPU_W) → fan-RPM surface and inverts it to a noise contour. Existing tools (CoolerControl, LACT, fw-fanctrl, TLP, GameMode) either drive fans directly or cap power blindly — none close the loop from *generated heat* to *acoustic outcome*. You are building something genuinely new; the closest conceptual cousins are RAPL power-capping controllers and RTSS-companion apps like DynamicFPSLimiter that throttle to a power/thermal budget.
- **Use a saturating, physically-structured model, not pure bilinear**: RPM ≈ affine in power near the middle of the range, but the underlying V/F and thermal-coupling physics are nonlinear. A recommended few-parameter form is a sum of per-device saturating (softplus/log) temperature terms plus a shared-heatsink cross term plus an ambient offset, fit from ~10–12 calibration points and adapted online with recursive least squares (RLS)/EWMA.
- **Respect time-scale separation and bound the integrator**: laptop thermal→fan settling is tens of seconds, so run the GPU-clock inner loop at ~1 Hz, the allocator at ~0.1–0.2 Hz (every 5–10 s), and the ambient trim integrator far slower (time constant minutes). Cap the trim integrator's cumulative authority hard so a mis-fit model degrades to "fans slightly loud," never "performance collapses."

## Key Findings

1. **GPU actuator reality**: On Blackwell mobile GPUs, `nvidia-smi -pl` (power limit) is unavailable — NVIDIA removed laptop power-limit control after driver 535 (a user report on the NVIDIA Developer Forums states verbatim: "Since the driver has updated to 535, Nvidia removed the ability to control the gpu (frequency, voltage and power), it was working in 525"), and the community-recommended workaround is exactly what you plan: `nvidia-smi --lock-gpu-clocks=min,max`. Because power scales super-linearly with clock along the voltage-frequency curve (P ≈ αf³ + static), a max-clock lock is a surprisingly effective indirect power cap, especially above the "ridge point" (~70–80% of peak clock).
2. **CPU actuator reality**: `ryzenadj` exposes STAPM (sustained), slow (average PPT), and fast (burst PPT) limits, which are moving-average limiters with configurable time constants — exactly the "shape sustained heat, leave bursts alone" primitive you want. On the Framework 16, the AMD Ryzen AI 9 HX 370's sustained power is governed by platform profile: per Framework's specs, Performance = 45 W sustained / 54 W boost, Balanced = 40 W / 48 W, Efficiency = 30 W / 36 W, and AMD lists the HX 370 with a configurable TDP (cTDP) range of 15–54 W.
3. **Prior art on power-shaping**: RAPL is the canonical power-cap-with-time-window mechanism; GameMode already does a crude version of demand-aware power balancing (switching CPU governor to powersave when the iGPU is under load, gated by an iGPU-Watts/CPU-Watts ratio); RTSS/DynamicFPSLimiter shows framerate-based dynamic power/thermal shaping works in practice.
4. **Framework-specific**: fw-fanctrl is the dominant community fan tool but it is a pure temperature→RPM curve driver (via ectool/framework_tool on the EC); it does NOT model heat/power. Its two tuning knobs — a 30 s moving average and a 5 s update interval, explicitly chosen "for comfort" — are directly reusable design lessons.
5. **Time constants**: Laptop CPU-die temperature responds to power in seconds, but system/skin/heatsink temperature and the resulting EC fan response settle over tens of seconds to minutes; treat ~30–90 s as the per-operating-point settling time for calibration.
6. **Model-trust and graceful degradation** are the crux: because the fan curve lives in the EC and you only shape heat, the failure mode of a bad model is either fans-too-loud (acceptable) or power-starved-performance (unacceptable). Bounding integrator authority and adding sanity/watchdog checks is what keeps failures on the acceptable side.

## Details

### 1. Prior art — what exists, what worked, what hunted

**No existing tool does noise-targeted power shaping via a learned heat→RPM model.** The design space divides cleanly into (a) fan-curve controllers, (b) power/clock cappers, and (c) framerate-based throttlers. Your design fuses the *outputs* of (b)/(c) to a *target* borrowed from (a) — that combination appears novel.

**Fan-curve controllers (drive fans directly — the thing you are *not* doing):**
- **CoolerControl** — feature-rich Linux daemon (coolercontrold) with graph profiles, hysteresis, thresholds, directionality and response-time tuning. Notably, on laptops "fan control on Linux is limited by what the kernel driver exposes"; many laptop fans appear only as read-only hwmon. Version 2.0 added CPU/GPU power display for supported devices. Relevant lesson: it exposes exactly the smoothing/hysteresis/directionality knobs you'll need, but it targets fan duty, not heat.
- **fw-fanctrl** (TamtamHero) — the canonical Framework tool. It "measures the CPU temperature, computes a moving average of it, and then finds an appropriate fan speed value by interpolation on the curve," using framework_tool/ectool to write the EC. Per its README, the "actual update to fan speed is made every 5s by default. This is for comfort, otherwise the speed is changed too often and it is noticeable and annoying, especially at low speed," and the temperature moving average "Defaults to 30 seconds." It does NOT model power or heat. **Two reusable lessons**: (i) heavy temporal smoothing before actuation to prevent audible hunting; (ii) EC access on Framework is via framework_tool/ectool (DHowett's ectool is the Windows-port hardware-comms layer).
- **LACT** — Linux GPU tool; supports NVIDIA 900-series and newer, with per-pstate clock offsets, power-limit configuration where the driver allows, and clockspeed control. Confirms that on NVIDIA "min and max values always have to be set together" for clock locking — the exact `--lock-gpu-clocks=min,max` semantics you'll drive. Also documents the power-profiles-daemon conflict you must avoid.

**Power/clock cappers (shape heat — your actuator layer):**
- **ryzenadj** — STAPM limit (`-a`, sustained), slow limit (`-c`, average PPT, governed by slow-time `-d`), fast limit (`-b`, burst PPT). Documented ordering constraint: "Fast Limit > Slow Limit > STAPM Limit." These are moving-average limiters, so leaving `fast-limit` high while lowering `slow`/`stapm` gives precisely "bursts unrestricted, sustained shaped." Caveat: on some OEM platforms values may be clamped or overwritten by platform firmware (the wiki notes STAPM "Gets overwritten by STTv2"), and some laptops refuse to raise above OEM caps (see the Ryzen 7 8840U issue #374 report of being "stuck at 25 W for continuous load") — sanity-check that your writes stick.
- **Intel RAPL / powercap** — the reference design for "average power over a time window." Each zone exposes a long-term constraint (sustained, with a time window commonly ~1–28 s) and a short-term constraint (burst, window ~2 ms / ~2441 µs in typical dumps). This is the intellectual template for your CPU actuator, though on AMD you use ryzenadj rather than the RAPL sysfs write path. Academic work (e.g., arXiv:2506.16046 on energy efficiency, arXiv:2308.08069 on RL power control) uses RAPL power caps as the actuator and confirms "RAPL then ensures the average power usage of the power zone does not exceed the power limit within the time window."
- **NVIDIA clock-lock as indirect power cap** — since driver 535 laptop `-pl` returns "Changing power management limit is not supported in current scope for GPU." The Arch community-recommended workaround (bbs.archlinux.org thread 302133): "Unfortunately, there's no way to explicitly limit GPU performance like there used to be, and Nvidia is quiet about this problem. I guess if you want to limit the power consumed by the GPU, you can try adjusting the clocks." Empirically a clock cap under-runs a nominal power limit (a tuning writeup notes a ~1050 MHz cap draws ~130–135 W even under a 150 W limit) and the GPU still runs *below* the locked ceiling under light load. This validates your GPU actuator choice. Note the Framework RTX 5070 module explicitly lacks Max-Q Dynamic Boost/WhisperMode.

**Framerate/thermal throttlers (Windows prior art worth mimicking):**
- **RTSS + DynamicFPSLimiter** — a companion app that reads GPU/CPU usage, power and temperature (via LibreHardwareMonitor) and "dynamically adjust[s] framerate limits," and can "define power and temperature constraints … across all detected GPUs." This is a working proof that a slow outer loop shaping a demand knob (framerate) to hit a power/thermal target is viable and pleasant. The frame-rate cap is a demand-side analog to your clock/power caps.
- **GameMode (Feral)** — most relevant feature: it detects when the integrated GPU is under load and switches the CPU governor to `powersave` (config `igpu_desiredgov`), gated by `igpu_power_threshold` — its example `gamemode.ini` sets `igpu_power_threshold=0.3` and describes it as "a ratio of iGPU Watts / CPU Watts … Set this to -1 to disable all iGPU checking" — because "the CPU and GPU share a thermal and power budget" and over-driving the CPU "can throw the graphics performance out of balance." Its reaper thread runs on `reaper_freq=5` ("check every 5 seconds … for the CPU/iGPU power balance"). **Two lessons**: (i) a Watts-ratio is a validated cheap CPU-vs-GPU demand signal; (ii) ~5 s is a sane cadence for the balancing decision. Caveat: GameMode's iGPU logic uses Intel RAPL and won't directly apply to your dGPU case, but the *principle* transfers.

**Academic / control-theory prior art:**
- Cascade control with time-scale separation is textbook: "always tune the inner loop first, then the outer loop"; the inner loop should be roughly 5–10× faster (MathWorks: "inner loop bandwidth should be ten times larger"; PLC practice: "each level needs to be 3–5× faster than the level above it"). Anti-windup for cascades: "when PID 2 saturates … inhibit PID 1 … If PID 1 is not inhibited, controller windup will occur." Back-calculation anti-windup for cascades is well documented (ACS Omega 2021, "Development of an Antiwindup Technique for a Cascade Control System").
- Fan-cooling controllers that build a thermal model note the cost you're avoiding: "thermal model construction requires a series of experiments for identifying parameters such as thermal resistance," and that transient thermal settling on server-class systems "may take a considerable amount of time, possibly up to 10 min or more, to reach a stable temperature" (Lee & Chen, *Sensors* 2015, PMC4481903). Laptops are faster (smaller mass) but the qualitative lesson — long thermal settling relative to your control step — holds.

**What hunted / failure lessons from the field**: fw-fanctrl's own documentation is essentially a warning that under-smoothed fan control is "noticeable and annoying"; the general laptop pattern of "sprinting, overheating, backing off, and then sprinting again" (thermal-throttle limit cycling, described in a MakeUseOf field report) is exactly the oscillation your slow, smoothed, deadbanded design must avoid. Perceptually, a fan held slightly high and *steady* is far preferable to one that pulses.

### 2. Model form — steady-state RPM as f(cpu_W, gpu_W)

**Physical structure.** The EC maps temperatures→RPM (with hysteresis); each device temperature is roughly its own power times a thermal resistance, plus coupling through the shared chassis, plus ambient. On the Framework 16 the CPU and GPU are on *separate* heatsinks — per Framework's specs the mainboard cooling uses "Two 6.0mm and one 8.0mm heatpipes" for the processor, while the RTX 5070 Graphics Module uses "Four 10mm heatpipes" with dual fans and supports "100W sustained TGP" (up to 100 W TGP on AC, up to 50 W on battery; the module has a 2.0 GHz base / up to 2.4 GHz boost clock, 4,608 CUDA cores, 8 GB GDDR7). This means CPU↔GPU thermal coupling is weaker than on shared-heatpipe gaming laptops — but not zero, because they share chassis air, intake, and airflow. Community reports confirm coupling exists: "using the GPU over the integrated graphics can help the CPU run a bit cooler," and mixed CPU+GPU loads change the CPU hotspot behavior versus CPU-only.

**Why pure bilinear is not enough.** Two nonlinearities matter:
- *V/F super-linearity on the GPU side*: because voltage must rise to sustain higher clocks, GPU dynamic power follows the canonical CMOS relation P_dyn = α·C·V²·f (confirmed for GPUs in the DVFS survey arXiv:1610.01784), and since V tracks f along the V/F curve, power scales roughly cubically with clock (modeled as P ≈ α·f³ + β static, arXiv:1512.07351). Measured V/F curves (arXiv:2211.07260) show a "ridge point" — "the clock frequency for the GPUs is 72% and 70% of the peak clock frequency for the Tesla A100 and RTX A4000 respectively" — below which voltage is flat (power ≈ linear in frequency; ~0.14 W/MHz measured on a Kepler K20, arXiv:1407.8116) and above which voltage climbs quadratically. For your model this matters mostly for the *clock→watts* mapping (Section 5), not the watts→RPM surface — but it means equal watt steps are NOT equal clock steps.
- *EC fan-curve saturation and knees*: fan RPM vs temperature is piecewise-linear with a floor (fan-off or minimum duty) and a ceiling (max RPM), plus hysteresis. So RPM vs power saturates at both ends.

**Recommended parametric form (few parameters, RLS-friendly).** Model each device's steady-state temperature as affine in its own power with a coupling term, then pass a soft-saturating link to RPM. A pragmatic, ~6–8 parameter model:

`RPM ≈ RPM_floor + softplus( a·cpu_W + b·gpu_W + d·min(cpu_W,gpu_W) + c − θ ) / s`

where `a`, `b` are per-device W→(fan-demand) sensitivities, `c` is an ambient/offset term, `d` captures shared-airflow coupling (a cross term; use `min(cpu_W,gpu_W)` or a product `cpu_W·gpu_W` — the min form is more robust with few points), `θ` is the fan-on knee, `s` scales, and `softplus(x)=ln(1+eˣ)` gives a smooth floor→linear transition. If you want to also capture the top-end RPM ceiling, wrap in a second saturating term or simply clamp.

**Simpler adequate fallback.** For a first implementation, an affine model with a cross term,
`RPM ≈ a·cpu_W + b·gpu_W + e·cpu_W·gpu_W + c`,
fit over the *operating region you actually care about* (mid-to-high power) is likely adequate, because within a bounded power window the saturating curve is approximately linear. Fit affine first; add the softplus/log saturation only if calibration residuals show systematic curvature at the extremes. This is the classic "linear-in-parameters" form ideal for recursive least squares.

**Heatsink coupling representation.** Because the FW16 uses separate heatsinks, model coupling as a single shared cross-term rather than a full 2×2 thermal-resistance matrix. If residuals demand more, escalate to a 2-temperature linear thermal model: `T_cpu = R_cc·P_cpu + R_cg·P_gpu + T_amb`; `T_gpu = R_gc·P_cpu + R_gg·P_gpu + T_amb`, with `RPM = max` over EC curves of each temperature. The off-diagonal R_cg, R_gc are the coupling; expect them small but nonzero. This is still linear-in-parameters and RLS-fittable.

**Online adaptation.** Keep parameters updated with either per-parameter EWMA on the offset `c` (fast, absorbs ambient drift — this is effectively your trim integrator) and slower RLS with forgetting factor λ≈0.98–0.995 on the slopes. Freeze slope updates unless the current operating point is informative (persistent excitation) and unless the system is near steady state (see loop cadence), to avoid learning from transients.

### 3. Loop design — cadences, time constants, anti-hunt

**Time constants (the governing physics).** After a power change: CPU die temperature moves within seconds; heatsink/skin and the EC's smoothed fan response settle over tens of seconds. fw-fanctrl's defaults (30 s moving average, 5 s update) encode the community's empirical estimate of this scale. Server-class thermal settling can be "up to 10 min or more"; laptops are faster but plan for **30–90 s to reach steady RPM** after a sustained power step. This dominates every cadence choice.

**Recommended three-tier cascade (inner fast → outer slow, with time-scale separation):**

- **(a) Inner GPU-clock→watts loop — ~1 Hz (1 s).** Purpose: hold GPU sustained watts at the allocator's setpoint by adjusting the max-clock lock. Because clock→watts is fast (electrical, sub-second) and monotone, a light PI (mostly I) suffices. Rate-limit clock changes (e.g., ≤ one V/F bin per second) and add a deadband (±2–3 W) so it doesn't chase NVML noise. The CPU side is simpler: write ryzenadj slow/stapm directly to the setpoint (open-loop, since ryzenadj already enforces the average internally); optionally trim with a slow correction if measured package watts drift from the limit.
- **(b) Allocator — every 5–10 s (~0.1–0.2 Hz).** Purpose: given the current ≤target-RPM contour and the current demand estimate (Section 4), pick the (cpu_W, gpu_W) operating point on the contour. This cadence is ~5–10× slower than the inner loop (satisfies time-scale separation) and matches GameMode's 5 s reaper and fw-fanctrl's 5 s update. Move setpoints with rate limiting and a deadband on the contour so small demand wobbles don't shuffle power back and forth audibly.
- **(c) Outer trim integrator — time constant of minutes (update every 10–30 s, tiny gain).** Purpose: shift the whole contour (the model's offset `c`) up/down to absorb ambient/airflow drift, by comparing *measured* RPM at the current operating point against the *model-predicted* RPM and integrating the error. Must be far slower than fan settling (≥ 3–5× the 30–90 s thermal time) or it will fight the thermal lag and hunt.

**Anti-hunt strategy (perceptual priority: steady-slightly-high beats oscillating):**
- **Asymmetric response**: raise power *slowly* (you're approaching the noise ceiling — creep up) and *cut power quickly* when measured RPM overshoots the target (protect the acoustic guarantee). This asymmetry (fast-down, slow-up on power) keeps you from audibly overshooting the noise target while avoiding twitchy upward corrections. Implement as different rate limits / different integrator gains per direction.
- **Deadbands** on RPM error (e.g., ±100–150 RPM, tuned to the just-noticeable-difference) and on power setpoints, so sub-threshold errors produce no actuation.
- **Rate limiting** on every actuator (clock bins/s, watts/s) so no single control step is audible.
- **Hysteresis** matching or exceeding the EC's own fan hysteresis, to avoid beating against it.
- **Anti-windup**: clamp each PI integrator and use back-calculation; critically, when the inner GPU loop saturates (clock at floor or ceiling) or ryzenadj clamps, *inhibit the allocator and trim integrator from winding further in that direction* — the cascade rule "when the inner loop saturates, inhibit the outer loop." This is the single most important stability safeguard.
- **Steady-state gating**: only let the trim integrator and RLS update when |dRPM/dt| and |dP/dt| are below thresholds (system near steady state), so you never learn from or correct during transients.

### 4. Demand estimation — cheap 1 Hz signals for CPU-bound vs GPU-bound

The allocator needs to know *which* device would benefit from more power. Compare candidate signals:

- **Utilization % (from /proc/stat for CPU, NVML `utilization.gpu` for GPU)** — cheap but weak. NVML "utilization" is only "the portion of time that the device is being used within the given sampling period, without considering the number of streaming multiprocessors (SMs)" (arthurchiao.art analysis) — it saturates at 100% long before the device is truly maxed, and can't tell "starved by our cap" from "genuinely maxed." Use as a coarse gate, not a fine signal.
- **Clock residency / whether clocks are pinned at your cap** — strong "starvation" detector. If the GPU is sitting exactly at your locked max-clock with high utilization, it is *starved by your cap* (raising it helps). If it's running *below* your cap, it's demand-limited (raising the cap won't help). Symmetrically for the CPU: if measured package watts are pegged at the ryzenadj slow/stapm limit, it's starved; if below, it's satisfied. This "am I pinned at the constraint?" test is the cleanest way to distinguish "starved by our cap vs genuinely idle."
- **Power drawn vs power allowed (RAPL package watts vs ryzenadj limit; NVML power.draw vs current cap)** — the best single demand signal, and the one GameMode uses (iGPU_W / CPU_W ratio). If a device is drawing ~100% of what you allow, it wants more; if it's drawing less, it doesn't. This directly gives the allocator its gradient: shift the marginal watt to whichever device is pinned at its cap.
- **Frame-time / FPS (if available via MangoHud/Vulkan layer)** — the ultimate arbiter of "did the user notice," but not always accessible; treat as optional enrichment.

**Recommended demand estimator**: for each device compute a "starvation" score = min(1, draw/cap) combined with a "pinned-at-clock/limit" boolean and utilization. Allocate the marginal watt (within the contour budget) to the device with the higher starvation score; if both are pinned, hold the split at the ratio that keeps both nearest their targets. Signals all come from `/proc/stat`, `/sys/class/powercap` (or amd_energy/RAPL) for CPU watts, cpufreq `scaling_cur_freq` for CPU clock residency, and NVML (`utilization.gpu`, `power.draw`, `clocks.sm`, `clocks.max`) for the GPU — all pollable at 1 Hz with negligible overhead (NVML counters, as one telemetry study notes, "sampled at 1 Hz … do not measurably burden or slow down the monitored workload").

### 5. Calibration experiment design

**Goal**: fit the ~6–12 parameters of Section 2 from a small, safe set of held operating points, recording steady-state RPM at each.

**Settling time per point**: hold each point until RPM is stable. Given laptop thermal settling of tens of seconds and EC smoothing, use **90 s hold, recording the mean RPM over the final 20–30 s** and requiring |dRPM/dt| below a threshold before accepting. If a point hasn't settled by 90 s, extend to 150 s. (Rationale: fw-fanctrl's 30 s average + margin; server settling can be minutes, laptops faster.)

**Controlling CPU watts**: set `ryzenadj --slow-limit` and `--stapm-limit` to the target sustained watts (and `--fast-limit` equal to avoid burst overshoot during calibration), then drive full load with `stress-ng --matrix 0` (the matrix stressor is documented as "an excellent way to make a CPU run hot" and gives a stable, high, repeatable floating-point load that will push the CPU to its power limit). Verify the CPU is actually *pinned at the limit* by reading RAPL/package watts — if measured < limit, the stressor isn't saturating (unlikely with matrix) or the limit isn't sticking. Sweep the limit to hit each target watt.

**Controlling GPU watts (the hard part — no power knob)**: since only max-clock lock is available, you must map clock→watts empirically. Procedure:
1. Run a *saturating* GPU load so the GPU is always pinned at the locked clock. `gpu-burn` (CUDA) is the most reliable saturator; if a graphics-style load is preferred, a looped `vkmark`/`glmark2` or FurMark/GpuTest — but note glmark2/vkmark "doesn't actually place any big load on the GPU" and may not saturate, so prefer gpu-burn or a Vulkan compute loop for calibration. gpu-burn alone may under-load some cards (a Level1Techs report saw only ~150 W on an A6000), so confirm the GPU is clock-pinned via NVML.
2. Step `--lock-gpu-clocks=C,C` across a range of C, and at each C record steady-state NVML `power.draw`. This yields your clock→watts curve (expect super-linear: big watt drops per MHz near the top, flattening toward the ridge point ~70–80% of peak). Store this as a lookup so the inner loop can invert watts→clock.
3. During the watts→RPM calibration, you then *command a target GPU watt* by looking up the corresponding clock.

**Concrete experiment matrix (≈11 points, ~20–30 min total):** (GPU high point targets the module's 100 W AC TGP ceiling; CPU high targets 45 W sustained / up to 54 W cTDP.)

| # | Purpose | CPU target | GPU target |
|---|---------|-----------|-----------|
| 1 | Baseline/ambient | idle (~5 W) | idle |
| 2 | CPU-only low | 15 W | idle |
| 3 | CPU-only mid | 30 W | idle |
| 4 | CPU-only high | 45 W | idle |
| 5 | GPU-only low | idle | ~35 W |
| 6 | GPU-only mid | idle | ~65 W |
| 7 | GPU-only high | idle | ~100 W (TGP) |
| 8 | Mixed low | 20 W | 40 W |
| 9 | Mixed mid | 30 W | 65 W |
| 10 | Mixed high | 45 W | 100 W |
| 11 | Mixed asymmetric | 45 W | 40 W |

Points 2–4 identify `a` (and CPU knee); 5–7 identify `b` (and the clock→watts curve); 8–11 identify the coupling term `d`/`e` and validate. Run in randomized order and interleave a return-to-baseline every few points to detect drift and hysteresis (approach some points from below and above to quantify EC hysteresis width). Log per-fan RPM, both temps, RAPL watts, NVML watts, GPU clock, and ambient (if a sensor exists) at 1 Hz throughout.

### 6. Failure modes and graceful degradation

The whole safety argument rests on one fact: **fans are on the EC's stock curve, so the EC will always cool the machine regardless of your controller.** Your controller can only make things *quieter than the EC would* by holding heat down; if your model is wrong, worst-case the EC just runs fans per its own (safe) curve. The danger is the *opposite* error: an over-aggressive trim integrator that keeps cutting power to chase a phantom noise target, collapsing performance. Design so that the acceptable failure (fans slightly loud) is the only reachable one.

**Ambient rise / blocked intake (blanket) / dust over months** all manifest identically: at a given (cpu_W, gpu_W) the measured RPM is *higher* than the model predicts (more RPM needed to shed the same heat), or temperatures rise toward throttle. The trim integrator will try to lower the contour (cut power) to bring RPM back to target. This is correct up to a point — but if intake is fully blocked, no amount of power cutting will hit the noise target without starving the machine.

**Bounded integrator authority (the key safeguard).** Cap the trim integrator's *total* cumulative correction to a hard limit — e.g., it may lower the effective power budget by at most X% (say 20–30%) below the model's nominal contour, and never below a **performance floor** (a user-set minimum sustained CPU W and GPU clock that guarantees playable performance). Concretely:
- Saturate the integrator state at ±max_trim (back-calculation anti-windup so it doesn't wind past the clamp).
- Enforce hard floors: cpu_W ≥ cpu_W_min, gpu_clock ≥ clock_min, independent of the integrator. These floors are the "graceful degradation" contract: once hit, the controller *stops cutting power and simply lets the fans run louder than target*, which is the acceptable failure.
- Surface a status when the floor is hit ("noise target not achievable — fans above target") so the user knows to check for a blanket/dust rather than silently losing performance.

**Model-trust monitors / sanity checks:**
- **Residual monitor**: track prediction error (measured RPM − model RPM). If the residual exceeds a threshold persistently, *reduce trust*: freeze slope adaptation, shrink integrator gain, and widen deadbands. A large sustained residual means the model no longer describes reality (dust, new thermal paste, ambient extreme).
- **Direction/plausibility checks**: reject RLS/EWMA updates that would make slopes negative or non-physical (more power → less RPM), or that exceed rate limits. Clamp parameters to physically plausible ranges.
- **Watchdog**: if temperatures approach throttle (e.g., Tctl → 95 °C / GPU → 87 °C) *despite* your caps, immediately abandon the noise target, release caps toward the performance floor, and let the EC fans take over — thermal safety always outranks the acoustic goal. Also watchdog the actuators themselves: verify ryzenadj writes stick (they can be overwritten by platform firmware/STTv2) and that clock locks are applied; if an actuator is unresponsive, fail safe to stock (remove caps).
- **Startup/faults**: on daemon crash or exit, remove all caps (ryzenadj back to defaults, `nvidia-smi --reset-gpu-clocks`) so the machine reverts to stock behavior — mirror fw-fanctrl's "if the service is paused or stopped, the fans revert to their default behaviour."

**Safe-default fallback behavior summary**: (1) never cut below performance floors; (2) on persistent model distrust, degrade to fans-above-target rather than power-below-floor; (3) on thermal emergency, release caps; (4) on actuator failure or crash, revert to stock. Every failure path leads to "a bit louder" or "stock behavior," never "collapsed performance."

## Recommendations

**Stage 1 — Instrument and characterize (before any control).** Build the 1 Hz logger (per-fan RPM via ectool/hwmon, CPU/GPU temps, RAPL/package W, NVML power/clock/util). Run the Section 5 calibration matrix, *including the clock→watts sweep first* (it's a prerequisite for commanding GPU watts). Fit the affine-plus-cross-term model; inspect residuals. **Go/no-go threshold**: if an affine+cross model fits with residuals under ~150–200 RPM across the matrix, proceed with it; only if residuals show systematic curvature at the extremes, upgrade to the softplus/2-temperature form.

**Stage 2 — Open-loop contour + inner loops.** Implement the GPU clock→watts inner PI (1 Hz, deadbanded, rate-limited) and direct ryzenadj CPU setpoints. Invert the model to the ≤target-RPM contour and run the allocator at 5–10 s with the Section 4 demand estimator. Do NOT enable the trim integrator yet. Validate against real games; confirm measured RPM stays at/below target and no audible hunting. **Threshold to advance**: RPM within deadband of target ≥90% of the time, zero perceptible fan oscillation over a 30-min session.

**Stage 3 — Enable the bounded trim integrator.** Add the slow (minutes-time-constant) offset integrator with hard saturation and performance floors from Section 6. Test the three failure scenarios deliberately: (i) raise ambient (run near a heater / in a sunny room), (ii) put it on a blanket, (iii) simulate dust by taping part of the intake. Confirm the controller degrades to "fans above target, performance floor held," never to starved performance. **Threshold**: in all three abuse tests, sustained CPU W and GPU clock never drop below floors, and the status flag correctly reports "target not achievable."

**Stage 4 — Online adaptation + polish.** Turn on RLS slope adaptation (gated to steady-state, forgetting λ≈0.99) and residual-based trust reduction. Add the watchdogs and safe-exit handlers. Ship with conservative defaults; expose the noise target, performance floors, and max-trim as user settings.

**Benchmarks that would change the plan**: if the clock→watts curve turns out non-monotone or hysteretic (some driver/vBIOS behavior), the inner GPU loop must switch from PI to a table-lookup + small trim. If CPU↔GPU coupling residuals are large (unexpected on separate heatsinks), escalate to the 2-temperature thermal model. If ryzenadj writes don't stick on the HX 370 Framework board, you may be limited to CPU platform-profile / EPP knobs instead — verify early in Stage 1.

## Caveats

- **Blackwell-specific power/clock behavior is inferred, not measured.** The "-pl removed on laptop GPUs since 535" evidence comes from Pascal/Ampere/Ada mobile parts (GTX 1060, RTX 3060/4060) and the RTX 5070/5070 Ti mobile power-lock reports; it is a driver-policy change reasonably expected to persist on the RTX 5070 Laptop, but no Blackwell-specific verbatim confirmation that `--lock-gpu-clocks` behaves identically was found. **Verify on your unit in Stage 1.** *(First-principles/speculation flagged.)*
- **Numeric thermal-settling and W/MHz figures are order-of-magnitude.** The ~0.14 W/MHz linear-region figure is from a 2014 Kepler K20 (HPC), not Blackwell; the ridge point (~70–80%) is measured on A100/A4000, not the RTX 5070; the 30–90 s settling estimate is inferred from fw-fanctrl defaults and server-class data, not measured on the FW16. Treat all specific seconds/watts/RPM thresholds in this document as *starting points to be measured in Stage 1*, not validated constants. *(Speculation flagged.)*
- **Framework 16 separate-heatsink assumption**: the CPU (two 6.0 mm + one 8.0 mm heatpipes) and RTX 5070 Graphics Module (four 10 mm heatpipes, 100 W TGP) heatsinks are physically distinct per Framework's specs, which is why a weak single coupling term is recommended rather than a shared-heatpipe model. If a future shared-vapor-chamber module changes this, revisit the coupling structure.
- **ryzenadj on this platform**: whether STAPM/slow writes stick (vs being overwritten by Framework's EC/platform firmware or STTv2) is unverified for the HX 370 FW16 board specifically; several users report OEM clamping on other laptops. Confirm early.
- **Some cited "quiet gaming" tools are Windows-only** (RTSS/DynamicFPSLimiter, MSI Afterburner, ThrottleStop) and are cited as *design precedent*, not as components you'll run on Linux. The framerate-based-thermal-control claim rests on the DynamicFPSLimiter project description and could not be independently benchmarked.
- **GPU numeric example (1050 MHz → ~130–135 W)** comes from a mining/tuning writeup, not a controlled measurement; use it only as illustrative of the qualitative "clock cap under-runs power cap" behavior.
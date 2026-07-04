# Research prompts for bazerame-fans

Run each in a separate researcher session. Paste results into `docs/research/` as
`01-hardware.md`, `02-rust.md`, `03-control.md`.

---

## Prompt 1 — Hardware integration (Framework 16 / HX 370 / RTX 5070)

I'm building a Rust tool for a Framework 16 laptop (2025 refresh: AMD Ryzen AI 9 HX 370
"Strix Point", NVIDIA RTX 5070 Laptop GPU "Blackwell", running Fedora Linux). The tool
shapes CPU/GPU sustained power draw to keep fan noise below a user target. It never
overrides fan curves — it only reduces heat so the EC spins fans down on its stock curve.

Known facts from the machine: hwmon exposes `k10temp` (Tctl), `cros_ec` and
`framework_laptop` (fan1_input, fan2_input — two fans, both currently reporting ~1450 RPM),
`amdgpu` (iGPU power), and RAPL via `/sys/class/powercap/intel-rapl` with only `energy_uj`
(no writable constraints). `ryzenadj`, `ectool`, `fw-ectool`, `framework_tool` are installed.
`nvidia-smi` works (driver 610.x); `-q -d POWER` reports a 5–100W limit range, but Blackwell
mobile GPUs reportedly reject setting power limits (`-pl`).

Research and report on:

1. **Fan topology of the Framework 16 (2025) with the dGPU module**: how many fans total,
   which hwmon channels map to which fans, which fan(s) cool the CPU vs the GPU module.
   Does the dGPU expansion bay have its own fan and is it visible via cros_ec hwmon or
   elsewhere? What is max RPM per fan? Any known EC fan-curve behavior docs (temp thresholds,
   hysteresis)?
2. **CPU power limiting on Strix Point (HX 370)**: does `ryzenadj` work reliably on this
   APU generation? Exact semantics of `--stapm-limit`, `--slow-limit`, `--fast-limit`
   (units, moving-average windows, interactions). Known issues: firmware resetting limits,
   conflicts with amd-pstate-epp or power-profiles-daemon, values reverting on AC/battery
   events. Alternatives: `ryzen_smu` kernel module, Universal x86 Tuning Utility approaches,
   any sysfs-native interface on recent kernels for STAPM-class limits. How to *read back*
   current limits and actual sustained power (RAPL package energy_uj accuracy on Zen 5).
3. **GPU control on Blackwell mobile**: confirm `nvidia-smi -pl` is rejected; does
   `nvidia-smi -lgc <min>,<max>` (lock GPU clocks) work on RTX 50 mobile? Granularity of
   clock steps, whether persistence mode is needed, latency of applying a new lock, and
   whether NVML (`nvmlDeviceSetGpuLockedClocks`) is a better path than shelling out. Any
   other sane knobs (e.g., `nvidia-smi --lock-memory-clocks`, power-mizer settings)?
   Relationship between locked max clock and resulting power draw on dynamic loads.
4. **Interference sources**: what else on a typical Fedora install fights over these knobs
   (power-profiles-daemon, tuned, TLP, GameMode, nvidia dynamic boost daemon
   `nvidia-powerd`)? How does AMD "SmartShift"-like power sharing between iGPU/dGPU/CPU
   behave on this platform, if present?
5. **Safety**: consequences of setting CPU limits too low (watchdog concerns), how to
   restore all defaults (exact commands), and whether limits persist across suspend/resume
   (do we need to reapply after resume?).

Cite sources (Framework community forum, kernel docs, ryzenadj GitHub issues) and clearly
mark anything uncertain or version-dependent.

---

## Prompt 2 — Rust stack for a root TUI control-loop app

I'm building a single-binary Rust TUI app (runs under sudo) for Linux that: polls sysfs
hwmon + RAPL energy counters + NVIDIA GPU stats at ~1Hz, runs a control loop (learned
thermal model + PID-style feedback) that shells out to or links against actuators
(ryzenadj, NVML), renders a live TUI dashboard with time-series graphs (temps, watts,
RPM, clocks) and interactive controls (fan target slider, pause/resume control loop),
and persists a small learned-model state file.

Recommend, with current (2026) crate versions and maturity assessment:

1. **TUI**: ratatui vs alternatives; best widgets for scrolling time-series charts;
   recommended event-loop pattern for "1Hz sampler + control loop + 30fps-ish UI +
   keyboard input" (tokio? plain threads + channels? calloop?). Prior-art open-source
   ratatui apps with live system graphs worth studying (e.g., bottom, bandwhich).
2. **Sensors**: best way to read hwmon (direct sysfs reads vs `libmedium`/`lm-sensors`
   binding crates), computing watts from RAPL `energy_uj` deltas correctly (wraparound
   handling), and NVML bindings (`nvml-wrapper` — maintained? supports RTX 50 /
   driver 610?) vs parsing `nvidia-smi` output.
3. **Actuation**: is there a usable libryzenadj Rust binding or should we shell out to
   `ryzenadj`? For GPU clock locks: NVML call vs `nvidia-smi -lgc` subprocess. Trade-offs
   for a 1Hz actuation rate.
4. **PID / control**: mature PID crates (`pid`, others) vs hand-rolling ~50 lines;
   anything for simple online regression / recursive least squares usable for fitting a
   2-variable thermal model incrementally.
5. **Plumbing**: config + state persistence (serde + toml/json), structured logging that
   coexists with a TUI (file-based tracing), graceful-shutdown patterns that guarantee a
   restore-defaults path runs on SIGINT/SIGTERM/panic.

Prefer boring, maintained crates. Note anything about running TUIs under sudo (terminal
capability quirks, XDG dirs pointing at root's home).

---

## Prompt 3 — Control design & prior art for noise-targeted power shaping

Context: Framework 16 laptop (AMD HX 370 CPU + RTX 5070 mobile GPU, Linux). Goal: hold
fan noise at/below a user-set level on dynamic workloads (gaming) while losing minimal
performance. Actuators: CPU sustained-power limits (ryzenadj slow/stapm — moving-average
limiters; short bursts intentionally unrestricted) and GPU max-clock lock (power limits
unavailable on Blackwell mobile). Fans stay on the EC's stock curve; we only shape heat.
Sensors at 1Hz: per-fan RPM, CPU/GPU temps, CPU package watts (RAPL), GPU watts (NVML).

Planned architecture: learn f(cpu_W_sustained, gpu_W_sustained) → steady-state fan RPM,
seeded by a one-time guided calibration (CPU-only burn at several power levels, GPU-only
burn, a few mixed points), adapted online; invert it to a "≤ target RPM" contour; an
allocator picks the point on the contour matching current demand; a slow feedback
integrator shifts the contour to absorb ambient-temperature/airflow drift.

Research and advise on:

1. **Prior art**: existing tools that do noise- or temperature-targeted *power* shaping
   (not fan-curve control): e.g., anything in CoolerControl, LACT, TLP, tuned, GameMode,
   Universal x86 Tuning Utility, "quiet gaming" tooling, ThrottleStop-style Windows tools,
   academic/hobbyist writeups on laptop thermal-budget controllers. What worked, what
   oscillated, what's worth stealing? Also fw-fanctrl and Framework-specific projects —
   do any model heat instead of driving fans?
2. **Model form**: is steady-state fan RPM as a function of (cpu_W, gpu_W) well-approximated
   by something simple (bilinear, sum of per-device saturating curves + ambient offset)?
   The EC maps temps→RPM with hysteresis and each device's temp depends mostly on its own
   power plus shared-heatsink coupling. Recommend a parametric form with few parameters
   fittable from ~10 calibration points and amenable to online RLS/EWMA updates.
3. **Loop design**: cadences and time constants — fan RPM responds to power changes with
   what typical lag (tens of seconds)? Recommend update rates for (a) inner GPU clock→watts
   PID, (b) allocator, (c) outer trim integrator, and anti-windup / hysteresis strategies
   so the system doesn't audibly hunt (fans oscillating is worse than fans slightly high).
4. **Demand estimation**: cheap signals to detect CPU-bound vs GPU-bound at 1Hz
   (utilization vs clock-residency vs power-at-unconstrained-clock) to steer the allocator.
5. **Calibration experiment design**: minimal set of (cpu_W, gpu_W) burn points and
   durations to fit the model (steady-state settling time per point?), good Linux burn
   tools (stress-ng options for CPU at controlled power; what for NVIDIA GPU load —
   glmark2? gputest? a vulkan compute loop?), and how to control *watts* during
   calibration when our only GPU knob is clocks.
6. **Failure modes**: what happens when ambient rises mid-session, dust builds up, or the
   laptop sits on a blanket — how should the trim integrator's authority be bounded so a
   broken model degrades to "fans slightly louder" rather than "performance collapses"?

Cite sources; mark speculation clearly.

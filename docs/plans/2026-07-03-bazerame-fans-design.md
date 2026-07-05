# bazerame-fans — Design

Noise-targeted power shaping for the Framework 16 (2025, Ryzen AI 9 HX 370 + RTX 5070
Mobile) on Bazzite Linux. A single Rust TUI binary, run under sudo, that holds fan noise
at/below a user-set RPM target on dynamic workloads by shaping *sustained* CPU/GPU power.
It never commands fans — the EC's stock temperature-driven curve stays active as the
safety net, so the worst reachable failure is "fans louder than target", never
"laptop overheats" and never "performance collapses".

Research inputs: `docs/research/01-hardware.md`, `02-rust.md`, `03-control.md`,
`04-ratatui.md`.

## 1. Scope & verified hardware ground truth

Verified on this machine (2026-07-03), not assumptions:

| Capability | Result |
|---|---|
| GPU max-clock lock (`nvidia-smi -lgc` / NVML) | **Works** on this Blackwell mobile. 1,651 supported bins (~7.5 MHz), range 210–3090 MHz. Locked 1500 → measured 1492 MHz |
| GPU power limit (`-pl`) | Blocked by NVIDIA on mobile Blackwell → clock locking is the GPU actuator |
| CPU sustained power limit | `ryzenadj --slow-limit/--stapm-limit` (mW) enforced exactly: 20 W commanded → 20 W measured via RAPL under full 24-thread load |
| ryzenadj operational caveat | Works **only with the `ryzen_smu` module unloaded**. Bazzite ships ryzen_smu 0.1.7 without Strix Point PM-table support; its presence makes ryzenadj pick the kmod backend and fail (`Unable to get os_access Obj`). With the module unloaded, ryzenadj falls back to libpci SMN access, which works. App must `modprobe -r ryzen_smu` at startup (and restore on exit) |
| CPU limit readback | Broken on this APU (`ryzenadj -i` fails; PM table unavailable) → treat writes as open-loop and verify via RAPL measurement |
| Restore CPU stock limits | Toggling `/sys/firmware/acpi/platform_profile` (e.g. → low-power → back) makes firmware reassert stock limits. Verified |
| Sensors | Fan RPM ×2 (`framework_laptop`/`cros_ec` hwmon), Tctl (`k10temp`), CPU pkg watts (RAPL `energy_uj` deltas, root-only), GPU watts/temp/clocks/util (NVML), iGPU watts (`amdgpu` hwmon) — all at 1 Hz |

Platform notes (from research):
- Fans: shared CPU+GPU cooling assembly ("Graphics Module Fan" cools both). Fan1/fan2
  channel→physical mapping unknown; treat `max(fan1, fan2)` as the controlled variable.
  EC spin-up threshold ≈ 47 °C. Stock behavior restores automatically when we stop.
- Interference: `tuned`/PPD profile switches clobber SMU limits; `nvidia-powerd` manages
  Dynamic Boost (leave running; it only raises GPU power within our clock ceiling).
  Limits do NOT survive suspend/resume — must reapply.
- Framework profile baselines: Performance 45 W sustained / 54 W boost; Balanced 40/48;
  Efficiency 30/36. GPU TGP up to 100 W AC.

Out of scope: fan-curve control, battery-mode behavior (AC gaming is the use case),
daemon/systemd packaging (foreground process; controller/UI split keeps a daemon split
possible later).

User-visible contract: one primary knob (max fan RPM), two safety knobs (CPU watts floor,
GPU clock floor), pause/resume, one-time guided calibration mode.

## 2. Process & thread architecture

One process, four OS threads, all events funneled through channels
(bottom-style; no tokio). Channels: `crossbeam-channel` (`select!` + `tick`).

- **Sampler thread (1 Hz)** — reads all sensors into a `Sample` struct; broadcasts to
  controller and UI. Handles RAPL wraparound (`max_energy_range_uj` modulus). Detects
  suspend/resume via monotonic-clock jump → emits `Resumed`.
- **Controller thread** — the only place hardware writes happen. Consumes samples, runs
  the control cascade, executes UI commands (`SetFanTarget`, `Pause`, `SetFloors`,
  `StartCalibration`, `Quit`), emits `ControlStatus` (setpoints, applied limits, trim
  state, flags) to the UI. Single owner of actuators → single Drop guard.
- **Input thread** — blocking `crossterm::event::read()` → events to main.
- **Main/render thread** — TEA-style: single `Model` (sample ring buffers, latest
  `ControlStatus`, UI focus), one `update(&mut Model, Event)`, pure `view(&Model, Frame)`,
  ~100 ms render tick. `update` unit-testable; views via `TestBackend`.

Sampler stays separate from controller so calibration's long settle phases never
interrupt the 1 Hz sampling cadence.

Shutdown: any exit trigger (q, SIGINT/SIGTERM via signal-hook, panic hook) → controller
drains → RAII guard restores hardware → `ratatui::restore()`. Startup also resets to
stock unconditionally (protects against a previous SIGKILL'd run).

## 3. Control cascade

Three tiers with strict time-scale separation (fan response lags power by 30–90 s):

- **Inner GPU loop, 1 Hz**: PI (mostly I) holds GPU watts at its allocation by moving the
  locked max clock along the calibrated clock→watts LUT. Deadband ±3 W, rate limit
  (~1 V/F bin/s), back-calculation anti-windup. Lock always `(210 MHz floor, max_target)` —
  never lock the min up. Applied via NVML `set_gpu_locked_clocks`.
- **CPU: open loop**: `--slow-limit`/`--stapm-limit` = allocation; `--fast-limit` stays at
  stock (~53 W) so short bursts pass through untouched (STAPM's moving-average semantics
  give "bursts fine, sustained shaped" for free). Reasserted every 10 s to defend against
  tuned/PPD clobbers; RAPL verifies the limit sticks (measured ≫ limit → reapply + alert).
- **Allocator, every 5 s**: inverts the thermal model to the ≤target-RPM contour in the
  (cpu_W, gpu_W) plane; picks the point via per-device starvation scores
  (draw/allowed ratio + pinned-at-limit boolean + utilization). Deadbanded, rate-limited,
  asymmetric: cut power fast on RPM overshoot, raise slowly when below target.
- **Trim integrator, every 20 s, minutes-scale time constant**: shifts the model offset
  by integrating (measured − predicted) RPM at the current operating point. Absorbs
  ambient/airflow/dust drift. **Hard-bounded authority**: total correction ≤ 25% budget
  reduction, and never below user floors (defaults: CPU ≥ 15 W, GPU clock ≥ 1000 MHz).
  Floor hit → status flag "target not achievable", fans allowed to exceed target.
  Only updates near steady state (|dRPM/dt| and |dP/dt| below thresholds).

**Thermal model**: start affine + cross term
`RPM = a·cpu_W + b·gpu_W + e·cpu_W·gpu_W + c`,
fit by RLS with forgetting λ≈0.99, updates gated to steady-state samples, slope-sanity
clamps (never negative). Upgrade to the softplus saturating form only if calibration
residuals exceed ~150–200 RPM. Controlled variable: `max(fan1, fan2)`.

**Watchdogs**: Tctl ≥ 95 °C or GPU ≥ 87 °C → release all caps (thermal safety outranks
acoustics). Actuator-stickiness check via RAPL/NVML. Suspend/resume → full reassert.

## 4. Calibration mode (guided, in-TUI)

1. **GPU clock→watts sweep** (prerequisite for commanding GPU watts): user starts a
   saturating GPU load (game/benchmark); the app verifies the GPU is pinned at each
   locked clock before recording (clocks.sm ≈ lock and utilization high), steps the lock
   across the range, records steady-state NVML watts → LUT.
2. **~11-point (cpu_W × gpu_W) matrix** (idle/low/mid/high per device + mixed points):
   built-in CPU burner threads (in-process spinners, verified to pin at the ryzenadj
   limit) + user-run GPU load. 90 s settle per point (extend to 150 s if |dRPM/dt| high),
   record mean RPM over final 20–30 s. Randomized order with return-to-baseline
   interleaves to detect drift/hysteresis.
3. Fit model, report residuals (go/no-go: <150–200 RPM → affine form OK), persist
   model + LUT + stock-limits snapshot to state file.

## 5. Safety & lifecycle

- Startup: unload `ryzen_smu` if its pm_table nodes are missing; enable GPU persistence
  mode; record stock state; unconditional reset-to-stock.
- Restore on every exit path (RAII Drop + panic hook + signal-hook):
  NVML `reset_gpu_locked_clocks`, CPU via platform-profile toggle, reload `ryzen_smu`,
  terminal restore.
- Floors enforced independently of the integrator. Every failure path degrades to
  "louder fans" or "stock behavior".
- CPU limit floor also hard-clamped at 10 W absolute (below user floor) to avoid
  UI stalls / resume instability.

## 6. Stack & module layout

| Concern | Choice |
|---|---|
| TUI | `ratatui` 0.30 + crossterm (no terminfo → sudo-safe) |
| Threads/channels | `std::thread` + `crossbeam-channel` |
| NVIDIA read+write | `nvml-wrapper` (in-process NVML; device by PCI bus ID) |
| CPU actuation | shell out to `ryzenadj` binary (git build installed; bindings are stale) |
| Sensors | direct sysfs reads (hwmon, RAPL) |
| PID | `pid` crate (anti-windup, derivative-on-measurement) |
| RLS | hand-rolled (~30 lines, `nalgebra` for 4×4 covariance) |
| Config | `serde` + `toml`, `/etc/bazerame-fans/config.toml` (CLI-overridable) |
| State (model, LUT, stock snapshot) | `serde_json`, `/var/lib/bazerame-fans/state.json`, atomic write (tmp+rename) |
| Logging | `tracing` + `tracing-appender` → file, `with_ansi(false)` |
| Signals/panics | `signal-hook`, panic hook chained with `ratatui`'s |
| CLI | `clap`; errors: `color-eyre` |

```
src/
├── main.rs, cli.rs, errors.rs, logging.rs
├── event.rs           # Event/Command enums
├── model.rs           # UI Model + update()
├── sensors/           # hwmon.rs, rapl.rs, nvml.rs, sampler.rs (thread)
├── actuators/         # cpu.rs (ryzenadj), gpu.rs (NVML locks), guard.rs (restore)
├── control/           # thermal_model.rs (RLS), allocator.rs, gpu_pid.rs,
│                      # trim.rs, controller.rs (thread), watchdog.rs
├── calib/             # runner.rs (state machine), burner.rs (CPU load threads)
├── state.rs           # persistence
└── ui/                # view.rs, charts.rs, panels/
```

TUI layout: time-series charts (fans RPM + target line; CPU/GPU watts with applied
limits overlaid; temps; GPU clock + lock), status bar (mode, trim %, model residual,
flags), keybound controls (target ±, pause, floors, calibrate).

Testing: unit tests for `update`, model fit/inversion, PID behavior, allocator policy
(synthetic samples); `TestBackend`/insta for views; sensor parsers against fixture
strings.

## 7. Milestones (each independently runnable)

1. **Monitor** — sensors + TUI dashboard, read-only. Verifies all numbers & charts live.
2. **Manual actuation** — set CPU watts / GPU max clock from the TUI; full safety/restore
   plumbing (guard, signals, startup reset, ryzen_smu handling).
3. **Calibration** — clock→watts sweep + matrix + model fit + persistence.
4. **Closed loop** — inner GPU PI + allocator against the fan target (trim disabled).
5. **Adaptive** — trim integrator, RLS online updates, watchdogs, resume-reassert, polish.

Acceptance (from research staging): RPM within deadband of target ≥90% of a 30-min
gaming session with zero perceptible fan oscillation; abuse tests (blanket, blocked
intake) degrade to flagged "target not achievable" with floors held.

## Post-field addendum (2026-07-05): why online RLS is off by default

Field session findings (telemetry run-1783230*): online RLS walked the cross-term `e`
negative until the contour divisor `b + e·pc` collapsed (~0.18 at pc=28), making the
model claim GPU watts were acoustically free; the allocator froze at high GPU power
with fans over target. Root cause is structural, not a tuning bug:

1. The steadiness gate (20-sample ≤100 RPM spread) measures spread, not slope — a slow
   monotonic climb passes while mid-transient.
2. Stale pairing: fans lag power by 30–90 s but Auto moves the power point every 5 s,
   so RLS pairs *current* commanded watts with RPM reflecting watts from ~30–60 s ago.
   Under an allocator ramp this systematically teaches "watts up, RPM unchanged" —
   i.e. exactly the degenerate direction observed.
3. RLS and trim double-correct the same residual; when RLS absorbs it into the shape,
   the contour bends; trim (offset-only, hard-clamped ±400 RPM) cannot corrupt
   invertibility.

Empirical confirmation: MODEL DISTRUST mode (RLS frozen, trim-only) produced the best
control of the session. Consequently `online_rls` defaults to false; the calibrated fit
owns the shape, trim owns drift, MODEL DISTRUST is a recalibration hint. A correct
online RLS would need the operating point held for the full fan-lag horizon (~45 s)
before a sample counts — a condition real gameplay almost never satisfies.
Guards added regardless (divisor floor in rls_update/fit_batch, model-independent
overshoot backstop in the allocator) so no adaptation path can reproduce the failure.

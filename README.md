# bazerame-fans

`bazerame-fans` shapes CPU and GPU heat on a Framework 16 so fw-fanctrl can hold a chosen fan-noise target. fw-fanctrl remains the only process that commands the fans. This app adjusts the CPU sustained-power cap and GPU clock ceiling, and restores stock limits whenever control stops or the process exits.

Auto uses one shared target temperature (T*) from fw-fanctrl's active curve and two independent controllers. The CPU controller regulates the CPU EC sensor group in watts; the GPU controller regulates the GPU EC sensor group in MHz. Each controller combines a thermal candidate with a measured shadow cap, applies its own slew limits, and can keep operating when the other device is unavailable.

## Current machine and tuning

This project is fine-tuned for our **Framework Laptop 16 (2025)** with an **AMD Ryzen AI 9 HX 370** (integrated Radeon 890M) and the **NVIDIA GeForce RTX 5070 Laptop GPU** graphics module, running Bazzite. The sensor grouping, actuator ranges, and calibration below come from this specific hardware and cooling setup; they are not a universal Framework 16 profile.

Current settings, recorded on September 12, 2026:

| Setting | Value |
|---|---|
| fw-fanctrl strategy | `quiet16` |
| Fan target | **3,500 RPM** |
| Fan-speed update interval | **1 second** |
| Temperature moving average | **30 seconds** |
| CPU sustained cap range | **15–54 W** |
| CPU short-burst PPT limit | **53 W** |
| GPU clock-cap range | **1,000–3,090 MHz** |

The current fan target resolves to approximately **79°C T*** with our learned duty/RPM table. T* is derived at runtime, not a fixed temperature setting.

The `quiet16` entry in fw-fanctrl's `/etc/fw-fanctrl/config.json` → `strategies` is:

```json
{
  "quiet16": {
    "fanSpeedUpdateFrequency": 1,
    "movingAverageInterval": 30,
    "speedCurve": [
      {
        "temp": 0,
        "speed": 20
      },
      {
        "temp": 55,
        "speed": 20
      },
      {
        "temp": 65,
        "speed": 24
      },
      {
        "temp": 75,
        "speed": 33
      },
      {
        "temp": 83,
        "speed": 48
      },
      {
        "temp": 87,
        "speed": 67
      },
      {
        "temp": 90,
        "speed": 100
      }
    ]
  }
}
```

Our app settings in `/etc/bazerame-fans/config.toml` include:

```toml
fan_target_rpm = 3500.0
cpu_floor_w = 15.0
cpu_max_w = 54.0
fast_limit_mw = 53000
gpu_floor_mhz = 1000
gpu_max_mhz = 3090
gpu_hot_c = 88.0
nvme_hot_c = 80.0
```

Latest saved calibration for **`quiet16:30`**:

| Device | Kc | Ti |
|---|---:|---:|
| CPU | 0.279475988 W/°C | 29.809252495 s |
| GPU | 46.435961060 MHz/°C | 48.413195881 s |

These are fitted values in `/var/lib/bazerame-fans/state.json`, not compiled defaults or TOML overrides. Calibration keeps gains separate for each strategy and averaging interval; the older 60-second fits remain separate. Press `k` to fit the active setup, keeping GPU load above 90% with a workload such as `gpu_burn`.

## Requirements

- Framework 16 (2025), Ryzen AI 9 HX 370 and RTX 5070 Mobile. The shipped limits and sensor grouping are specific to this machine.
- Bazzite or a compatible Fedora-family system with the proprietary NVIDIA driver.
- Root access for RAPL, `ryzenadj`, NVML clock locks, and `ryzen_smu` module handling.
- A recent `ryzenadj` build in `PATH`.

Bazzite's packaged `ryzen_smu` lacks Strix Point PM-table support. The app unloads it before using `ryzenadj`'s working SMN backend and reloads it on exit. A failed unload leaves monitoring available and reports CPU actuation failure.

## Setup and usage with fw-fanctrl

Run **fw-fanctrl alongside this app**. fw-fanctrl reads the EC temperatures and sets fan duty using its active curve. `bazerame-fans` reads that curve and adjusts CPU power and GPU clock caps to approach the requested fan RPM; it does not set fan duty itself.

1. Install fw-fanctrl and `framework_tool`, and ensure fw-fanctrl is running. If your installation supplies `fw-fanctrl.service`, start it with `sudo systemctl enable --now fw-fanctrl`. Otherwise, run `sudo fw-fanctrl run` in a separate terminal.
2. Add the **`quiet16` entry above** to the `strategies` object in `/etc/fw-fanctrl/config.json`, preserving the rest of that file. Set `defaultStrategy` to `quiet16` if it should be the startup strategy. This profile uses a **30-second moving average** and a **1-second update interval**.
3. Reload and select the curve, then inspect the active configuration:

   ```sh
   sudo fw-fanctrl reload
   sudo fw-fanctrl use quiet16
   sudo fw-fanctrl print all
   ```

   Check that the strategy is `quiet16`, control is active, and the curve and averaging interval match the profile above. The app's default read-only socket is `/run/fw-fanctrl/.fw-fanctrl.commands.sock`.
4. Build the app and create `/etc/bazerame-fans/config.toml` with the settings above:

   ```sh
   cargo build --release
   sudo ./target/release/bazerame-fans
   ```

5. Press **`k` to calibrate** your machine. Keep GPU utilization above 90% throughout calibration; `gpu_burn` is the suggested workload. Follow the on-screen CPU-load prompts, and wait for the explicit success or partial-success result. The saved gains apply only to that strategy and moving-average interval.
6. Press **`Esc`** to dismiss the result, then **`a`** to engage Auto. Start the workload you want to quiet. The header shows the mode, fan target, T*, per-device temperature errors, caps, and gain source. Use **`t` / `T`** to lower or raise the fan target by 250 RPM. Allow time for the machine to warm up and settle; reaching the target depends on the workload and cooling conditions.
7. Press **`p`** to release the caps and return to Monitor, or **`q`** to quit and restore stock limits. fw-fanctrl continues running and managing the fans.

The app can monitor and run Auto with default gains before calibration, but the current profile above uses measured fits. Recalibrate after changing the fw-fanctrl strategy or averaging interval. Select the curve before starting calibration; calibration freezes its context for the run.

Optional paths are selected with `--config`, `--state-file`, `--telemetry-dir`, and `--log-dir`. `sudo ./target/release/bazerame-fans selftest` exercises the sensor and actuator paths before a controlled session.

## Configuration

The TOML file accepts partial configuration. Unknown legacy keys are ignored during migration.

| Key | Default | Meaning |
|---|---:|---|
| `fan_target_rpm` | `3000` | Requested steady fan speed |
| `cpu_floor_w` | `15` | Lowest CPU sustained cap Auto may request |
| `gpu_floor_mhz` | `1000` | Lowest GPU clock ceiling Auto may request |
| `fast_limit_mw` | `53000` | CPU short-burst PPT limit |
| `cpu_max_w` | `54` | CPU sustained ceiling |
| `gpu_max_mhz` | `3090` | GPU clock ceiling |
| `shadow_headroom_cpu_w` | `10` | CPU shadow headroom over measured draw |
| `shadow_headroom_gpu_mhz` | `300` | GPU shadow headroom over measured clock |
| `shadow_fall_rate_cpu` | `0.33` | CPU shadow downward slew in W/s |
| `shadow_fall_rate_gpu` | `10` | GPU shadow downward slew in MHz/s |
| `gpu_shadow_enabled` | `true` | Enables the GPU measured-shadow candidate |
| `cpu_hot_c` | `90` | CPU hot-guard entry temperature |
| `cpu_gains` | unset | Optional CPU `{ kc, ti_s }` override |
| `gpu_gains` | unset | Optional GPU `{ kc, ti_s }` override |
| `gpu_hot_c` | `88` | GPU hot-guard entry temperature |
| `nvme_hot_c` | `80` | Reporting threshold for the NVMe sensor |
| `fanctrl_socket` | `/run/fw-fanctrl/.fw-fanctrl.commands.sock` | Read-only fw-fanctrl command socket |
| `leds` | enabled | Optional LED matrix settings |

The `[leds]` table supports `enabled`, `cpu_port`, `gpu_port`, `brightness`, `flip_time`, `cpu_flip_watts`, and `gpu_flip_watts`.

## Controls

| Key | Action |
|---|---|
| `a` | Toggle Auto |
| `t` / `T` | Fan target −/+ 250 RPM |
| `c` / `C` | Manual CPU cap −/+ 2 W |
| `g` / `G` | Manual GPU ceiling −/+ 105 MHz |
| `f` / `F` | CPU floor −/+ 1 W |
| `d` / `D` | GPU floor −/+ 105 MHz |
| `p` | Release limits and return to Monitor |
| `k` | Start guided calibration |
| `Esc` | Abort calibration, or dismiss its completed result |
| `q` / `Ctrl-C` | Quit and restore stock limits |

## Calibration

Calibration starts with a shared steady hold, then performs a native CPU-watt step and a native GPU-clock step. Keep GPU load above 90% throughout; `gpu_burn` is suggested for generating that load. Each response is fitted independently and stored under the fw-fanctrl strategy and moving-average interval that produced it. A rejected device fit keeps the existing keyed gain or uses the default. Calibration uses the normal verified actuator paths, stops cleanly on abort, and never commands the fans. Movement of the other device’s temperature no longer rejects a fit: shared cooling is part of the measured response.

After calibration, the result remains visible until Esc or another calibration. Use Up/Down to scroll long explanations and Home to return to the top. It reports success, partial success, or failure, with each device’s old/new Kc and Ti or its rejection reason and retained gains. Save failures are explicit; configured gain overrides still take precedence and are identified in the result.

The GPU runtime default now uses the 2026-09-12 recorded fit: Kc ≈ 22.2218 MHz/°C and Ti ≈ 32.5641 s at a 60-second moving average. Other intervals adjust the delay contribution; CPU defaults are unchanged. Calibration still accepts Kc only within 0.25×–4× the corresponding default. CPU cap maintenance is shared by normal operation and calibration: every 10 seconds it reads back the slow/fast limits, rewrites a confirmed mismatch, and verifies the repair. Matching caps and unreadable checks do not trigger blind CPU writes. Calibration also checks before completing a response; a reset discards that response, restores the baseline pair, settles again, and retries the affected device up to twice. Loss of readable verification during a response fails explicitly. Guards and release/shutdown ownership still take precedence. The new default passes the recorded-model response check, but produced 11.12°C overshoot on the previous provisional GPU model (K=0.02, τ=15 s, delay=90 s). The existing broad simulation matrix uses an explicit older gain and does not validate this new default; real-machine runs have since supplied the interval-specific fitted gains documented above; the default-model simulation is not a substitute for validating a new hardware setup.

The state file stores the duty/RPM relation, keyed CPU and GPU gains, paired warm starts, and a qualified last-good T*. Old state keys are detected and ignored with migration warnings.

## Auto control

```text
fan target -> snapped duty -> fw-fanctrl curve -> shared T*
                                      |-> CPU group PI -> CPU watts cap
                                      `-> GPU group PI -> GPU MHz ceiling
```

A valid curve supplies T*. When the curve is temporarily unavailable, the source holds and slowly adjusts the last trustworthy target. When a known board sensor (ambient or charger) is hottest, Held mode continues regulating CPU/GPU power using fan RPM feedback; the sensor label does not establish that its heat is independent of CPU/GPU power. Missing device-group data holds only that device; a missing GPU does not perturb CPU decisions.

Measured draw or clock supplies a shadow candidate that follows available work. The thermal candidate takes over when a device is above the current T*, including when T* itself falls. Hot guards ratchet each device's allowed maximum toward its configured floor and reopen only after measured recovery. Actuator mismatches hold the affected device for reassertion; three confirmed failures release stock limits. Suspend/resume clears stale timing and evidence before caps are reasserted.

`NOT CALIBRATED` is informational and means fitted gains are absent for the active strategy/interval. Safe defaults still allow Auto to operate.

## Safety

- fw-fanctrl owns fan commands and its EC curve remains the thermal safety net.
- The fw-fanctrl client can issue only the read commands `print speed` and `print all`.
- Thermal-emergency and CPU-sensor-loss watchdogs release all limits and require acknowledgement before actuation resumes.
- CPU and GPU writes use paired read-back evidence. Repeated confirmed mismatches restore stock behavior; unreadable evidence is reported separately.
- GPU heat changes the GPU ceiling. NVMe heat is reported because measurements showed fan-target changes were not an effective NVMe control action.
- Startup, normal exit, signals, panics, and controller-thread failure all restore stock CPU and GPU behavior.

## Telemetry

While calibration is running, a `kind: "calibration"` record is emitted for every processed calibration sample, even when the UI status is unchanged. Schema v4 introduced this record; v5 adds CPU cap read-back evidence. Existing sample/decision shapes remain unchanged.

- `phase` is the phase evaluated on this sample; `next_phase` records any transition (`settle`, `cpu_step`, `gpu_recovery`, `gpu_step`, `done`). Ordinary settle/response timeout samples retain their final gate snapshot before the runner is removed. Command aborts and global watchdog exits remain decision/flag events.
- `run_elapsed_s`, `phase_elapsed_s`, and `timeout_s` give timing. `gates` contains the seven actual settle checks with `satisfied`, `observed`, `last_observed_at_s`, and continuous satisfied/failed `duration_s`.
- `windows` exposes actual sample counts, `coverage_s`, `required_s`, measured peak-to-peak `span`, `limit`, and `unit`. CPU and GPU limits are 1°C and 2°C over 60 seconds; fans remain 150 RPM over 20 seconds. The window retains one sample at or before its time boundary so sampling jitter cannot prevent full coverage; that boundary sample counts toward the span. A small span is insufficient if window coverage is too short. Empty windows have null span. Gates/windows are null during response phases so old settle results cannot masquerade as current measurements.
- `context.cpu_cap_checked_at_s` timestamps the latest CPU cap read-back. `cpu_cap_readback` preserves the result before any repair; `cpu_cap_reset_reason` marks the tick that detected a reset, even if repair succeeded. The command completion timestamp remains separate.
- `context` records the exact controller inputs to calibration, including CPU/GPU cap verification and command completion times, group moving averages, EC mismatch, controllable argmax, and fanctrl activity. `baseline_cpu_w`, `baseline_gpu_mhz`, and `phase_commanded_at_s` expose the held-pair comparison.
- `gpu_util_pct`, `view_fresh`, `view_changed`, `reconciliation_ma_c`, and `socket_ma_c` provide verification and EC-reconciliation context. Missing values are null. Completion times use the same monotonic seconds as `t_mono`.


Schema v5 JSONL contains `run_start`, 1 Hz `sample`, event-driven `decision`, and flag-transition records. Samples include raw CPU/GPU/fan data, fw-fanctrl data, reconciliation values, and independent `cpu_group_c`/`gpu_group_c` readings. Decisions include T* state, per-device group/error, thermal and shadow candidates, selected candidate, hold reason, gains source, applied CPU watts/GPU MHz, and structured diagnostic flags.

## Files

- `/etc/bazerame-fans/config.toml` — operator configuration
- `/var/lib/bazerame-fans/state.json` — calibrated gains and qualified entry state
- `/var/lib/bazerame-fans/telemetry/` — JSONL runs
- `/var/lib/bazerame-fans/log/` — tracing logs

`cargo test` runs the offline suite. Hardware tests are ignored and must be invoked manually. The current design is [Per-device temperature loops, revision 4](docs/superpowers/runs/2026-09-11-per-device-temperature-loops/2026-09-11-per-device-temperature-loops-design.md).

The TUI groups each device’s temperature error with its cap (W / GHz) in the top line, alongside target temperature/state, fan target, ambient, and NVMe temperature. Fitted gains have no label; defaults and config overrides show red `unfitted`. Error is target minus group temperature: amber ↑ means below target, amber ↓ means above target, and green ≈ means zero at the displayed 0.1°C precision. Arrows indicate the desired direction, not an observed trend or a guarantee that the workload will reach the target. Ambient is green below T* and amber at or above it. NVMe is green below the drive’s `temp1_max` hwmon threshold (read at startup), and amber at or above it; unavailable or invalid thresholds fall back to configured `nvme_hot_c` (80°C by default). This display threshold does not change the existing `NVME HOT` warning threshold. Missing temperatures or unknown comparison targets are gray. Warnings precede control details; narrow terminals clip the right side of this single line.

Board temperature no longer imposes an ambient-plus-5°C lower bound on T*. The upper thermal ceiling remains enforced. A hot episode now transfers thermal control from the applied cap whenever the device is above the current target with shadow selected, including target-only crossings; five cool seconds re-arm it. A settled fan-target limitation is reported after both devices remain at their minimum caps for a full minute with CPU/GPU spans ≤1/2°C, fan span ≤150 RPM, and fan speed >150 RPM above target. The top status line also shows board ambient and NVMe temperatures.

Hot handoff can now skip unused headroom: with five seconds of valid recent draw, it seeds the thermal limit at the smaller of the applied cap and recent peak draw + 2 W (CPU) / 100 MHz (GPU), or the configured shadow margin if smaller. It trims once per hot episode, respects hardware floors and GPU slew from the actually applied cap, and does not follow brief draw dips. Normal shadow headroom is unchanged. Auto and calibration follow fw-fanctrl’s live averaging interval. Saved fits remain keyed by strategy and interval; if the active key has no fit, interval-adjusted defaults apply until calibration succeeds.

Chart target overlays are historical series sampled alongside measurements: fan RPM target, temperature T*, CPU watts cap, and GPU clock cap. Status updates change future points without rewriting earlier history; absent or released temperature/cap targets leave gaps. The histories cover the same rolling five-minute window and begin when the app starts.

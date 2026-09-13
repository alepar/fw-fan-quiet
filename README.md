# fw-fan-quiet

`fw-fan-quiet` shapes CPU and GPU heat on a Framework 16 so fw-fanctrl can hold a chosen fan-noise target. fw-fanctrl remains the only process that commands the fans. This app adjusts the CPU sustained-power cap and GPU clock ceiling, and restores stock limits whenever control stops or the process exits.

In Auto mode, you choose how fast you want the fans to spin. The app uses fw-fanctrl’s fan curve to work out the temperature to aim for, then adjusts the CPU and GPU separately: if either gets too hot, it lowers that device’s limit; if it has room to warm up, it lets the device do more work. It makes these changes gradually and accounts for how much work each device is actually doing. If one device’s readings disappear, it can still manage the other.

## Example settings

Below are example settings tuned for a **Framework Laptop 16 (2025)** with an **AMD Ryzen AI 9 HX 370** (integrated Radeon 890M) and the **NVIDIA GeForce RTX 5070 Laptop GPU** graphics module, running Bazzite. Use them as a starting point, not a universal profile. Built-in recalibration (`k`) can fit the controller gains to other Framework laptop models and cooling setups; sensor mappings and hardware limits still need to match the machine.

The example uses these settings:

| Setting | Value |
|---|---|
| fw-fanctrl strategy | `quiet16` |
| Fan target | **3,500 RPM** |
| Fan-speed update interval | **1 second** |
| Temperature moving average | **30 seconds** |
| CPU sustained cap range | **15–54 W** |
| CPU short-burst PPT limit | **53 W** |
| GPU clock-cap range | **1,000–3,090 MHz** |

With this setup, a 3,500 RPM fan target corresponds to a temperature target (**T***) of about **79°C**. The app calculates T* from the active fan curve and the measured relationship between fan duty and RPM. It is not a fixed temperature setting.

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

Example app settings in `/etc/fw-fan-quiet/config.toml` include:

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

Example calibration for **`quiet16:30`** (the `quiet16` strategy with a 30-second moving average):

| Device | Kc | Ti |
|---|---:|---:|
| CPU | 0.279475988 W/°C | 29.809252495 s |
| GPU | 46.435961060 MHz/°C | 48.413195881 s |

Kc controls how strongly the app responds to a temperature error. Ti sets how quickly it corrects an error that persists. Calibration measures these values and saves them in `/var/lib/fw-fan-quiet/state.json`. Each fan strategy and averaging interval has its own saved gains. Press `k` to calibrate your setup.

## Requirements

- Linux with fw-fanctrl running and compatible Framework EC temperature/fan readings exposed through hwmon.
- An AMD CPU supported by `ryzenadj`, with readable CPU package-energy counters.
- For GPU control: an NVIDIA GPU and driver exposing NVML clock-lock support. Other GPU control backends are not implemented.
- Root access for package-energy readings, CPU power limits, GPU clock locks, and any required kernel-module handling.
- A recent `ryzenadj` build in `PATH`.

No particular Linux distribution is required by the code. The setup above has been tested on Bazzite with the listed Framework 16 hardware. Other configurations need matching sensor mappings and hardware limits; recalibration fits the thermal response but does not discover those mappings or limits automatically.

The app includes a `ryzen_smu` compatibility workaround: if the loaded module exposes no PM table, it attempts to unload it so `ryzenadj` can use its alternate backend, then reloads it on exit. A failed unload is reported, and CPU actuation may remain unavailable.

## Setup and usage with fw-fanctrl

Run **fw-fanctrl alongside this app**. fw-fanctrl reads the EC temperatures and sets fan duty using its active curve. `fw-fan-quiet` reads that curve and adjusts CPU power and GPU clock caps to approach the requested fan RPM; it does not set fan duty itself.

1. Install fw-fanctrl and `framework_tool`, and ensure fw-fanctrl is running. If your installation supplies `fw-fanctrl.service`, start it with `sudo systemctl enable --now fw-fanctrl`. Otherwise, run `sudo fw-fanctrl run` in a separate terminal.
2. Add the **`quiet16` entry above** to the `strategies` object in `/etc/fw-fanctrl/config.json`, preserving the rest of that file. Set `defaultStrategy` to `quiet16` if it should be the startup strategy. This profile uses a **30-second moving average** and a **1-second update interval**.
3. Reload and select the curve, then inspect the active configuration:

   ```sh
   sudo fw-fanctrl reload
   sudo fw-fanctrl use quiet16
   sudo fw-fanctrl print all
   ```

   Check that the strategy is `quiet16`, control is active, and the curve and averaging interval match the profile above. The app's default read-only socket is `/run/fw-fanctrl/.fw-fanctrl.commands.sock`.
4. Build the app and create `/etc/fw-fan-quiet/config.toml` with the settings above:

   ```sh
   cargo build --release
   sudo ./target/release/fw-fan-quiet
   ```

5. Press **`k` to calibrate** your machine. Keep GPU utilization above 90% throughout calibration; `gpu_burn` is the suggested workload. Follow the on-screen CPU-load prompts, and wait for the explicit success or partial-success result. The saved gains apply only to that strategy and moving-average interval.
6. Press **`Esc`** to dismiss the result, then **`a`** to engage Auto. Start the workload you want to quiet. The header shows the mode, current and target fan speed, T*, device temperature errors, and caps. Use **`t` / `T`** to lower or raise the fan target by 250 RPM. Allow time for the machine to warm up and settle; reaching the target depends on the workload and cooling conditions.
7. Press **`p`** to release the caps and return to Monitor, or **`q`** to quit and restore stock limits. fw-fanctrl continues running and managing the fans.

Auto can use built-in gains before calibration. Calibrate to fit its response to your machine. Recalibrate after changing the fan strategy or averaging interval, and keep both unchanged during calibration.

Optional paths are selected with `--config`, `--state-file`, `--telemetry-dir`, and `--log-dir`. `sudo ./target/release/fw-fan-quiet selftest` exercises the sensor and actuator paths before a controlled session.

## Configuration

You can specify only the settings you want to change. Missing keys use defaults; unknown keys are ignored.

| Key | Default | Meaning |
|---|---:|---|
| `fan_target_rpm` | `3000` | Requested steady fan speed |
| `cpu_floor_w` | `15` | Lowest CPU sustained cap Auto may request |
| `gpu_floor_mhz` | `1000` | Lowest GPU clock ceiling Auto may request |
| `fast_limit_mw` | `53000` | CPU short-burst PPT limit |
| `cpu_max_w` | `54` | CPU sustained ceiling |
| `gpu_max_mhz` | `3090` | GPU clock ceiling |
| `shadow_headroom_cpu_w` | `10` | CPU power allowance above measured use |
| `shadow_headroom_gpu_mhz` | `300` | GPU clock allowance above measured use |
| `shadow_fall_rate_cpu` | `0.33` | Rate at which the CPU workload-based cap falls, in W/s |
| `shadow_fall_rate_gpu` | `10` | Rate at which the GPU workload-based cap falls, in MHz/s |
| `gpu_shadow_enabled` | `true` | Limits unused GPU clock headroom |
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

Calibration measures how temperature responds to a change in each device’s limit. It waits for temperatures and fans to settle, changes the CPU power cap, and then tests a change in the GPU clock cap. These measurements set the strength and timing of each controller’s response.

Keep GPU utilization above 90% throughout the run. `gpu_burn` is a suggested load generator. Follow the CPU-load prompts on screen. Keep the fan curve and averaging interval unchanged so the measurements describe one consistent setup.

The result reports success, partial success, or failure. It shows which gains changed, which were retained, and why any measurement was rejected. Each device is evaluated separately, so a successful CPU calibration can be saved even if the GPU measurement fails. Rejected measurements retain the saved gains for that setup, or use built-in gains if none are saved. Calibration accepts Kc between 0.25× and 4× its corresponding built-in value.

If a CPU cap resets during a measurement, the app repairs it, discards the affected measurement, and retries after settling. It allows up to two retries per device. It reports an error if it cannot verify the cap or save the results. Explicit gain overrides in the configuration take priority over saved calibration.

Press `Esc` to abort a run or dismiss its result. Use Up/Down to scroll a long result and Home to return to its top.

## How Auto works

The fan curve relates temperature to fan duty. The app also learns how fan duty relates to RPM. Together, these give it a temperature target for your requested fan speed. The header calls this target **T***.

Each device has two limits:

- A **temperature-based limit** decreases when the device is too hot and increases when it is below target. A proportional-integral (PI) controller responds both to temperature changes and to errors that persist.
- A **workload-based limit** follows measured CPU power or GPU clock use, with spare capacity for short bursts. The code calls this the *shadow* limit.

The app selects the lower limit, within the configured minimum and maximum. Changes are rate-limited to avoid abrupt shifts. CPU and GPU limits are adjusted separately because the devices respond differently to heat and load.

A cap above actual demand has little cooling effect. When a device becomes hot, the app can skip this unused headroom using the highest draw measured over five seconds, plus 2 W for CPU or 100 MHz for GPU. The allowance is smaller if you configured less headroom. If Auto starts while a device is already hot, it waits for those readings before making one such adjustment. This avoids spending minutes lowering an ineffective cap. It does not repeatedly follow short drops in workload.

When the fan curve is temporarily unavailable, or a board sensor is hottest, **Held** mode uses the last usable target and fan-speed feedback. Board temperature can respond to CPU and GPU heat, so both device controllers remain active. Missing device-temperature data holds that device’s control; the other device can continue operating.

The fan target is a goal, not a guarantee. Workload, cooling conditions, and the configured minimum caps can prevent the app from reaching it. If both devices are at their minimum caps and settled fans remain too fast, the app reports that the target cannot be reached.

`NOT CALIBRATED` means there are no saved gains for the active strategy and averaging interval. Auto can still use its built-in gains.

## Reading the display

The header shows temperature errors and current/configured-maximum caps, for example:

```text
CPU ↓-3.7°C - 44.0/54W | GPU ↑+0.6°C - 2.8/3.1GHz | fan 3123/3750 rpm
```

Temperature error is T* minus the device’s measured temperature:

- **Cyan ↑**: below target, with room to warm up.
- **Orange ↓**: above target, so cooling is needed.
- **Green ≈**: at target to the displayed precision of 0.1°C.

The arrows show the desired direction, not the observed temperature trend. A red `unfitted` label means the device uses built-in gains or a configuration override rather than saved calibration.

Fan RPM is the faster of the two fans. It is green below the requested speed and orange at or above it. Ambient temperature is green below T* and orange at or above it. NVMe temperature uses the drive’s reported maximum temperature (`temp1_max`); if that is unavailable, it uses `nvme_hot_c`. This display color is separate from the `NVME HOT` warning, which always uses the configured threshold. Missing readings are gray.

Graphs show a rolling five-minute history. Gray lines record the targets at each point in time. A bright **`<`** on each right border marks the current target, so it remains visible when a measurement covers the target line. Released or unavailable targets have no marker.

The shared top-right legend and target markers use cyan for fans, yellow for CPU, and green for GPU. The shared temperature target is white. Narrow terminals prioritize warnings and clip the right side of the status text.

## Limits and recovery

fw-fanctrl controls the fans throughout operation. This app reads its configuration and status using `print all` and `print speed`; it does not send fan-control commands.

CPU and GPU hot guards reduce the affected device’s allowed maximum until it cools. Emergency-temperature and CPU-sensor-loss watchdogs release the limits and require acknowledgement before control resumes. NVMe heat is reported; it does not change CPU or GPU limits.

The app checks that requested caps take effect. Every 10 seconds, it reads back the CPU power limits and repairs a confirmed reset. It verifies the result after a repair. Unreadable checks are reported and do not trigger blind repair writes. Repeated confirmed failures release control and restore stock limits.

After suspend, the app clears stale timing and measurements before reapplying caps. Release, shutdown, signal, panic, and controller-failure handling attempt to restore stock CPU and GPU behavior. Errors are recorded in the log.

## Telemetry and troubleshooting

Each run writes JSONL telemetry with a schema version and timestamps. The records explain what the app measured and why it changed a cap:

- **Samples** contain CPU/GPU temperatures, power and clock readings, fan RPM, and fw-fanctrl status.
- **Decisions** contain T*, device temperature errors, both candidate limits, the selected caps, and reasons for holding control.
- **Calibration records** contain the active phase, elapsed time, settling checks, and cap-verification results. They are written for each processed calibration sample.
- **Flag changes** record warnings and state transitions.

For a slow or stalled calibration, inspect both the temperature variation and the observation-window coverage. Settling requires CPU variation within 1°C and GPU variation within 2°C over 60 seconds, plus fan variation within 150 RPM over 20 seconds. A quiet but incomplete window is not yet settled.

For slow cooling, compare the selected cap with actual use. A falling cap will not reduce heat while it remains above what the workload needs. Decision records also show whether temperature control, workload headroom, a guard, or missing data is limiting progress.

## Files

| Path | Contents |
|---|---|
| `/etc/fw-fan-quiet/config.toml` | User settings |
| `/var/lib/fw-fan-quiet/state.json` | Saved calibration and learned control settings |
| `/var/lib/fw-fan-quiet/telemetry/` | JSONL run records |
| `/var/lib/fw-fan-quiet/log/` | Diagnostic logs |

Use `--config`, `--state-file`, `--telemetry-dir`, and `--log-dir` to choose other paths.

## Development

Run `cargo test` for the offline test suite. Hardware tests are ignored by default and require an explicit run. `cargo clippy --all-targets -- -D warnings` checks the code.

For implementation details, see the [controller design](docs/superpowers/runs/2026-09-11-per-device-temperature-loops/2026-09-11-per-device-temperature-loops-design.md).

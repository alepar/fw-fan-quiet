# bazerame-fans

`bazerame-fans` shapes CPU and GPU heat on a Framework 16 so fw-fanctrl can hold a chosen fan-noise target. fw-fanctrl remains the only process that commands the fans. This app adjusts the CPU sustained-power cap and GPU clock ceiling, and restores stock limits whenever control stops or the process exits.

Auto uses one shared target temperature (T*) from fw-fanctrl's active curve and two independent controllers. The CPU controller regulates the CPU EC sensor group in watts; the GPU controller regulates the GPU EC sensor group in MHz. Each controller combines a thermal candidate with a measured shadow cap, applies its own slew limits, and can keep operating when the other device is unavailable.

## Requirements

- Framework 16 (2025), Ryzen AI 9 HX 370 and RTX 5070 Mobile. The shipped limits and sensor grouping are specific to this machine.
- Bazzite or a compatible Fedora-family system with the proprietary NVIDIA driver.
- Root access for RAPL, `ryzenadj`, NVML clock locks, and `ryzen_smu` module handling.
- A recent `ryzenadj` build in `PATH`.

Bazzite's packaged `ryzen_smu` lacks Strix Point PM-table support. The app unloads it before using `ryzenadj`'s working SMN backend and reloads it on exit. A failed unload leaves monitoring available and reports CPU actuation failure.

## Build and run

```sh
cargo build --release
sudo ./target/release/bazerame-fans
```

Optional paths are selected with `--config`, `--state-file`, `--telemetry-dir`, and `--log-dir`. Run `sudo ./target/release/bazerame-fans selftest` once to exercise the sensor and actuator paths before a controlled session.

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
| `Esc` | Abort calibration |
| `q` / `Ctrl-C` | Quit and restore stock limits |

## Calibration

Calibration starts with a shared steady hold, then performs a native CPU-watt step and a native GPU-clock step. Each response is fitted independently and stored under the fw-fanctrl strategy and moving-average interval that produced it. A rejected device fit keeps the existing keyed gain or uses the safe default. Calibration uses the normal verified actuator paths, stops cleanly on abort, and never commands the fans.

The state file stores the duty/RPM relation, keyed CPU and GPU gains, paired warm starts, and a qualified last-good T*. Old state keys are detected and ignored with migration warnings.

## Auto control

```text
fan target -> snapped duty -> fw-fanctrl curve -> shared T*
                                      |-> CPU group PI -> CPU watts cap
                                      `-> GPU group PI -> GPU MHz ceiling
```

A valid curve supplies T*. When the curve is temporarily unavailable, the source holds and slowly adjusts the last trustworthy target. Known uncontrollable sensor dominance bypasses regulation without coupling the two devices. Missing device-group data holds only that device; a missing GPU does not perturb CPU decisions.

Measured draw or clock supplies a shadow candidate that follows available work. The thermal candidate takes over once measured heat crosses T*. Hot guards ratchet each device's allowed maximum toward its configured floor and reopen only after measured recovery. Actuator mismatches hold the affected device for reassertion; three confirmed failures release stock limits. Suspend/resume clears stale timing and evidence before caps are reasserted.

`NOT CALIBRATED` is informational and means fitted gains are absent for the active strategy/interval. Safe defaults still allow Auto to operate.

## Safety

- fw-fanctrl owns fan commands and its EC curve remains the thermal safety net.
- The fw-fanctrl client can issue only the read commands `print speed` and `print all`.
- Thermal-emergency and CPU-sensor-loss watchdogs release all limits and require acknowledgement before actuation resumes.
- CPU and GPU writes use paired read-back evidence. Repeated confirmed mismatches restore stock behavior; unreadable evidence is reported separately.
- GPU heat changes the GPU ceiling. NVMe heat is reported because measurements showed fan-target changes were not an effective NVMe control action.
- Startup, normal exit, signals, panics, and controller-thread failure all restore stock CPU and GPU behavior.

## Telemetry

Schema v3 JSONL contains `run_start`, 1 Hz `sample`, event-driven `decision`, and flag-transition records. Samples include raw CPU/GPU/fan data, fw-fanctrl data, reconciliation values, and independent `cpu_group_c`/`gpu_group_c` readings. Decisions include T* state, per-device group/error, thermal and shadow candidates, selected candidate, hold reason, gains source, applied CPU watts/GPU MHz, and structured diagnostic flags.

## Files

- `/etc/bazerame-fans/config.toml` — operator configuration
- `/var/lib/bazerame-fans/state.json` — calibrated gains and qualified entry state
- `/var/lib/bazerame-fans/telemetry/` — JSONL runs
- `/var/lib/bazerame-fans/log/` — tracing logs

`cargo test` runs the offline suite. Hardware tests are ignored and must be invoked manually. The current design is [Per-device temperature loops, revision 4](docs/superpowers/runs/2026-09-11-per-device-temperature-loops/2026-09-11-per-device-temperature-loops-design.md).

# bazerame-fans

Noise-targeted power shaping for the Framework 16, closing the loop through **fw-fanctrl**.
Auto mode holds the fans at a user-chosen RPM target by capping sustained CPU and GPU
power through a single PI integrator that regulates the same temperature fw-fanctrl's own
curve reacts to (Mode A), falling back to regulating measured fan RPM directly whenever
the temperature loop is unusable (Mode B). fw-fanctrl
remains the only process that ever commands the fans — this app only shapes the heat that
reaches its curve. The EC's stock temperature-driven fan curve stays untouched as the
safety net, so the worst reachable failure is "fans louder than target" — never "laptop
overheats", never "performance collapses".

A single Rust TUI binary. No daemon, no services: start it for a gaming session, quit it
when done, and every exit path restores stock hardware behavior.

## Requirements

- **Hardware**: Framework 16 (2025), Ryzen AI 9 HX 370 + RTX 5070 Mobile. The actuator
  ranges (CPU 10–54 W sustained, GPU clock lock 210–3090 MHz) and sensor topology are
  verified on exactly this machine; other hardware would need re-verification.
- **OS**: Bazzite (or another Fedora-family distro) with the NVIDIA proprietary driver
  (NVML is used for GPU sensing and clock locking).
- **Root**: required — RAPL energy counters, `ryzenadj`, NVML clock locks and module
  load/unload are all root-only.
- **`ryzenadj` in `PATH`**: a recent git build. Note the Strix Point caveat below.

### The `ryzen_smu` caveat

Bazzite ships the `ryzen_smu` kernel module (0.1.7) without Strix Point PM-table support.
With the module loaded, `ryzenadj` picks the kmod backend and fails
(`Unable to get os_access Obj`); with it unloaded, `ryzenadj` falls back to libpci SMN
access, which works. The app handles this itself: it runs `modprobe -r ryzen_smu` at
startup and reloads the module on exit. If the unload fails, monitoring still works but
CPU actuation will visibly fail.

## Build & run

```sh
cargo build --release
sudo ./target/release/bazerame-fans
```

Flags (all optional):

| Flag | Default | Purpose |
|---|---|---|
| `--config` | `/etc/bazerame-fans/config.toml` | User config (see Configuration below) |
| `--state-file` | `/var/lib/bazerame-fans/state.json` | Persisted calibration (clock→watts table, PI gains, duty↔RPM table) |
| `--telemetry-dir` | `/var/lib/bazerame-fans/telemetry` | JSONL telemetry logs |
| `--log-dir` | `/var/lib/bazerame-fans/log` | Tracing logs |

Subcommand: `sudo ./target/release/bazerame-fans selftest` — exercises actuators and
sensors end to end (~25 s, no TUI) and prints plain `[ OK ]`/`[FAIL]` lines. Run it once
before trusting the app with a session.

## Configuration

TOML at the `--config` path; every key is optional (a partial file overrides only what it
names, missing/corrupt files fall back to defaults, unknown keys are always ignored — a
config written by an older build stays loadable forever). The fan target and floors are
also editable live from the TUI and written back to this file.

| Key | Default | Meaning |
|---|---|---|
| `fan_target_rpm` | `3000.0` | Steady-state fan RPM Auto mode holds the machine at |
| `cpu_floor_w` | `15.0` | CPU sustained-watts floor: Auto never allocates below this |
| `gpu_floor_mhz` | `1000` | GPU locked-clock floor (MHz): Auto never locks below this |
| `fast_limit_mw` | `53000` | CPU fast (short-burst) PPT limit handed to ryzenadj |
| `cpu_max_w` | `54.0` | CPU sustained operating max (watts) — the "100%" CPU power the allocator grid-searches up to and the actuator clamps to; hard-clamped to the HX 370 cTDP ceiling. Lower it to soft-cap the CPU |
| `gpu_max_w` | `100.0` | GPU operating max (watts), same role for the GPU; hard-clamped to the RTX 5070 module TGP |
| `gpu_hot_c` | `90.0` | dGPU guard enter threshold (°C, exit is this − 5). See Safety model below |
| `nvme_hot_c` | `80.0` | NVMe guard enter threshold (°C, exit is this − 5). Reporting-only — see Safety model below |
| `fanctrl_socket` | `/run/fw-fanctrl/.fw-fanctrl.commands.sock` | `AF_UNIX` socket for the fw-fanctrl client. This binary never writes to it |
| `leds` | see below | `[leds]` table for the optional LED matrix wattage display |

`[leds]` table (all optional; a missing/failed module just stays dark):

| Key | Default | Meaning |
|---|---|---|
| `enabled` | `true` | Master switch for the whole LED feature |
| `cpu_port` | `/dev/serial/by-path/pci-0000:c4:00.0-usb-0:4.2:1.0` | Serial device for the CPU (left) gauge |
| `gpu_port` | `/dev/serial/by-path/pci-0000:c4:00.0-usb-0:3.3:1.0` | Serial device for the GPU (right) gauge |
| `brightness` | `100` | Global PWM brightness sent to both modules (0–255) |
| `flip_time` | `false` | Reverse the time axis (set if newest ends up at the bottom) |
| `cpu_flip_watts` | `true` | Reverse the CPU (left) panel's wattage-bar growth direction |
| `gpu_flip_watts` | `false` | Reverse the GPU (right) panel's wattage-bar growth direction |

## Keys

| Key | Action |
|---|---|
| `a` | Toggle Auto mode (the closed loop; needs a calibration first) |
| `t` / `T` | Fan target −/+ 250 RPM (clamped 1000–7000) |
| `c` / `C` | Manual CPU sustained limit −/+ 2 W (clamped 10–54 W) |
| `g` / `G` | Manual GPU max clock −/+ 105 MHz (clamped 1000–3090 MHz) |
| `f` / `F` | CPU power floor −/+ 1 W (clamped 10–54 W) |
| `d` / `D` | GPU clock floor −/+ 105 MHz (clamped 1000–3090 MHz) |
| `p` | Release all limits, back to Monitor mode (app keeps running) |
| `k` | Start guided calibration (from Monitor mode only) |
| `Esc` | Abort a running calibration |
| `q` / `Ctrl-C` | Quit (restores stock hardware state first) |

Floors are safety config, not actuation: they bound what Auto mode may ever take away
(defaults CPU ≥ 15 W, GPU ≥ 1000 MHz), are editable in any mode except during a
calibration, and persist to the config file immediately.

## Calibration walkthrough

Auto mode needs a one-time calibration (per machine/placement). Press `k` from Monitor
mode and follow the wizard panel through its two phases:

1. **GPU clock→watts sweep** (`lut` phase, 10 locked clocks from 3090 MHz down to
   1200 MHz, ~210 MHz apart). Start a saturating GPU-heavy load (game or benchmark) when
   the wizard shows `NEEDS GPU LOAD` — the sweep verifies the GPU is actually pinned at
   each locked clock (3 consecutive samples within 30 MHz of the lock, ≥90% utilization)
   before recording its steady-state watts into a clock→watts lookup table.
2. **Step test** (`step` phase). The app starts its own in-process CPU burner and holds
   the power budget at the configured floors until the EC temperature and fan RPM settle
   (flat within 0.5 °C over 60 s, 5-minute cap), then steps the total power budget up by
   30 W — split between CPU and GPU by demand, so keep providing GPU load if the wizard
   nags `NEEDS GPU LOAD` — and holds until the response is flat for 90 s (5-minute cap).
   A first-order-plus-dead-time model is fitted to both the EC-temperature response and
   the RPM response, and PI gains for Auto mode's two loops (below) are derived from the
   fit. The step aborts and skips (keeping the built-in default gains, with a reason noted
   in telemetry) if the EC temperature hits 95 °C, if the dominant heat sensor changes
   mid-step, if power never actually rose, or if the fitted response is too small or
   physically implausible to trust.
3. **Persist.** The clock→watts table, the fitted PI gains (or the defaults, if the step
   was skipped) and the duty↔RPM lookup table are written to the state file.

Expect roughly 10–20 minutes total, dominated by fan settle times. `Esc` aborts cleanly
at any point. Calibration never touches the fans directly — fw-fanctrl's own curve keeps
driving them throughout, exactly as it does outside a calibration.

## Auto mode

Press `a`. Every allocator tick (5 s) the cascade runs outermost-first:

```
target RPM ──(duty/RPM table)──▶ target duty ──(fw-fanctrl's live curve, inverted)──▶ T*
T* − EC average   [Mode A]   ─┐
target RPM − measured RPM [Mode B]   ─┴──▶ one PI (5 s) ──▶ power budget (W)
power budget ──(floors first, then by per-device demand)──▶ (cpu_w, gpu_w)
cpu_w ──▶ ryzenadj limit (+ read-back)      gpu_w ──▶ GPU PI (1 Hz) ──▶ NVML clock lock
```

There is exactly one integrator in this whole chain (fw-fanctrl's own curve is P-only),
which is what makes running two controllers off one measurement safe.

**The two modes.** The arbiter (`control/mode.rs`) picks which loop, if either, owns the
budget's integrator this tick, first row that matches wins:

| Mode | Header shows | Runs when |
|---|---|---|
| **Mode A — TempLoop** | `mode A` | fw-fanctrl's socket is fresh and `active`, the controller's own EC replica reading is valid and reconciled against fw-fanctrl's view (no `EC MISMATCH`), the dominant heat sensor is one this app can steer, and the target temperature (T\*) is feasible — held for 3 consecutive ticks (15 s) before switching in |
| **Mode B — RpmLoop** | `mode B` | Mode A's conditions aren't met but the fan tachometer reading is valid. Regulates measured RPM directly against the same duty-snapped target Mode A would use, so a switch between the two never jumps the setpoint |
| **Released** | `mode rel` | Neither loop has anything to close on (socket dead *and* fan reading invalid): caps release to stock |

A mode switch never resets the power budget — only the next PI increment changes
(bumpless by construction), so falling from Mode A to Mode B (socket dies, a strategy
edit, `active` flips false) never steps the commanded power.

Header flags you may see, in the order the TUI prioritizes them when several compete for
one line (most severe first):

| Flag | Meaning |
|---|---|
| `THERMAL EMERGENCY` | Tctl ≥ 95 °C or GPU ≥ 87 °C for 3 samples: everything released toward stock. Requires manual re-arm: the first actuating key (`a`, `c`/`g`, `k`) only acknowledges; the second acts |
| `SENSOR LOST` | CPU temperature unreadable for 10 samples while limits were applied: assume hot, same release + re-arm semantics |
| `TARGET UNREACHABLE` | The target temperature/RPM can't be reached: infeasible (too close to an uncontrollable sensor), held at the power floor for 60 s while still calling for less heat ("low"), or held at the ceiling for 60 s while still calling for more ("high") |
| `LIMIT-SLIP!` | RAPL keeps measuring above the commanded CPU limit; reasserting |
| `NOT CALIBRATED` | Auto was requested without a calibrated LUT — run `k` |
| `CURVE INVALID` | fw-fanctrl's resolved curve is non-monotone and rejected: Mode A is unavailable until the curve is fixed (does not clear on its own) |
| `EC MISMATCH` | The controller's EC replica disagrees with fw-fanctrl's own reported temperature for 3 consecutive checks: Mode A is unavailable until 3 consecutive checks agree again |
| `FANCTRL LOST` | The fw-fanctrl socket is absent or stale: Mode A is unavailable, the loop falls to Mode B. Informational while in Mode B; clears when the socket returns |
| `GPU HOT` | The dGPU is at/over `gpu_hot_c` (default 90 °C, exit 85 °C): its allocator share is ratcheted down toward its floor each tick |
| `NVME HOT` | The NVMe drive is at/over `nvme_hot_c` (default 80 °C): reporting only, see Safety model |
| `resumed` | Suspend/resume detected recently (clears after 30 s): limits were reasserted, GPU persistence re-enabled, and the read-back watchdog runs stricter for 60 s |
| `STEEP CURVE` | The target sits on a segment of fw-fanctrl's curve steeper than 2 %/°C. The loop still runs |
| `READBACK BLIND` | Six consecutive actuator read-backs came back unreadable/unverifiable; clears on the next verified one |

## Telemetry

Every run writes one JSONL file under the telemetry dir (falls back to the current
directory if unwritable). Schema v3 has one `run_start` header, then 1 Hz `sample`,
event-driven `decision`, and standalone `flag` lines. It loads directly into pandas
(`pd.read_json(path, lines=True)`) or DuckDB (`read_json_auto`) for offline review.

`sample` retains the raw fan, CPU/GPU, validity, resume, EC max/argmax/moving-average,
NVMe, and fw-fanctrl speed/active/strategy fields. It also has independent nullable
`cpu_group_c` and `gpu_group_c` values in °C: a missing dGPU group is `null` without
discarding the CPU group.

`decision` records the timestamp, top-level controller `mode`, `cpu_limit_w` (W),
`gpu_max_mhz` (MHz), `fan_target_rpm`, and `cause`. `t_star` (°C) is paired with
`tstar_state` (`curve`, `held`, `uncontrollable`, or `released`). The nullable `cpu` and
`gpu` objects each carry `group_c` and `err_c` (°C). CPU `thermal`, `shadow`, and `cap` are
watts; GPU `thermal`, `shadow`, and `cap` are MHz. `selected` is the JSON string
`thermal`, `shadow`, `floor`, or `max`. `hold` is a serde-tagged object: for example,
`{"kind":"shadow"}`, `{"kind":"group_unavailable"}`, or
`{"kind":"clamp","bound":"floor"}` (the bound is `floor` or `max`). Every object also
has `gains_source` (`config`, `fitted`, `default`). Until the controller wiring lands,
these three v3 fields are explicit `null`s while the legacy arbiter continues to emit its
decision values.

The `flags` array uses objects with a `name` and `active` polarity. Label-bearing flags
(`argmax_uncontrollable`, `argmax_stuck`, `ec_unknown_label`, `ec_implausible`) also carry
`label`; `group_lost` carries `device`; `device_unreachable` carries `device` and `bound`;
and `target_unreachable` carries `bound`. `ec_uncontrollable_unavailable` and `steep_curve`
need no extra detail. Existing pre-v3 controller flags appear temporarily as
`{"name":"legacy","flag":...,"active":true}`. A separate `flag` line remains the
transition stream with its string flag name and `active` polarity.

`demand_cpu`, `demand_gpu`, `alloc_cpu_w`, `alloc_gpu_w`, `pi_target_w`, `budget_w`, and
`freeze` are deprecated compatibility columns. They remain populated while the legacy
allocator is still live and are scheduled for removal in task `eb9.12`.

## Safety model

- **fw-fanctrl owns the fans, always.** This app never touches the EC fan curve or
  commands a fan directly; it only shapes how much heat the CPU/GPU generate, which
  fw-fanctrl's own curve then reacts to. Thermal safety always outranks acoustics — every
  failure path degrades to louder fans or stock behavior, never to heat.
- **The fw-fanctrl socket connection is read-only by construction.** The client's command
  enum has exactly two variants — `print speed` and `print all` — there is no code path in
  this binary that can send a `set`/`use`/`pause` command or any other write to the
  socket.
- **Guards** (`gpu_hot_c`/`nvme_hot_c`, both with a 5 °C exit hysteresis below the enter
  threshold): while the dGPU is at/over `gpu_hot_c` its allocator share is ratcheted down
  toward its floor every tick (`GPU HOT`). **The NVMe guard is reporting-only: a hot drive
  raises `NVME HOT` and is shown in the status line, but no control action is ever taken
  on it.** This is a measured decision, not an omission — under sustained I/O the drive
  climbed 67 → 80 °C in 30 seconds while the fans were already pinned near maximum, and
  the EC's own temperature reading *fell* from 74 to 69 °C over that same window because
  the load was I/O-bound and left the SoC idle. Near-maximum airflow did not hold the
  drive, and the guard's only possible lever — raising the fan target — would raise the
  regulated temperature (and therefore the CPU/GPU power budget), injecting more heat into
  a scenario that had nothing running that could use it.
- **Actuator read-back.** Every CPU limit write is verified by re-parsing `ryzenadj
  --info`; every GPU clock lock is verified against the measured SM clock while the GPU is
  loaded. A verified mismatch raises `LIMIT-SLIP!`, freezes the budget integrator, and
  forces an immediate reassert; three consecutive mismatches release everything to stock.
  Six consecutive unreadable/unverifiable read-backs (not themselves failures — e.g. the
  GPU is idle, or `ryzenadj --info` can't run) raise the informational `READBACK BLIND`.
- Every exit path (quit, signals, panics, even a controller-thread death) restores stock:
  GPU clock locks reset, CPU limits restored via a platform-profile toggle, `ryzen_smu`
  reloaded, terminal restored. Startup also unconditionally resets to stock, covering a
  previous SIGKILL'd run.
- Watchdogs: thermal emergency and sensor-lost release everything and latch until
  manually re-armed (deliberate two-step, see the flag table); a stickiness check
  verifies via RAPL that CPU limits actually hold; suspend/resume triggers a full
  reassert plus a 60 s strict-checking window.
- Floors bound the controller's authority independently of which loop mode is active.
- `selftest` exercises the whole actuation path before you rely on it.

## File locations

- `/etc/bazerame-fans/config.toml` — fan target, floors, power maxes, guard thresholds,
  the fw-fanctrl socket path (see Configuration; written back by the in-app editors;
  missing/corrupt files fall back to defaults, never crash)
- `/var/lib/bazerame-fans/state.json` — calibration state (GPU clock→watts table, fitted
  PI gains, duty↔RPM table, warm-start budgets)
- `/var/lib/bazerame-fans/telemetry/` — JSONL telemetry, one file per run
- `/var/lib/bazerame-fans/log/` — tracing logs

## Development

`cargo test` runs the whole suite (no hardware or root needed; hardware-touching tests
are `#[ignore]`d and run manually). The design/decision trail lives in
`docs/plans/` (design + task plan) and `docs/research/` (hardware ground truth, control
theory, stack notes).

Status: the fw-fanctrl closed-loop rework (this README's Auto mode section) is landing in
stages. The arbiter (`control/mode.rs`), the budget PI (`control/budget.rs`), the guards,
the socket client and the step-test calibration described above are all implemented and
unit-tested standalone; wiring them into the controller's live `on_auto_sample` path
(warm-start, passive refinement, and the calibration hand-off) is tracked as the epic's
remaining tasks. Until that wiring lands, `a` still drives the prior scalar-budget-split
allocator. The on-machine calibration run and a real gaming-session validation of Auto
mode are pending on top of that, per this project's usual field-validation pattern.

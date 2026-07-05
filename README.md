# bazerame-fans

Noise-targeted power shaping for the Framework 16. Instead of controlling the fans, it
holds fan noise at a user-chosen RPM target by shaping *sustained* CPU and GPU power —
the novelty is the direction of control: a calibrated thermal model of *your* machine is
inverted to the "≤ target RPM" contour in the (CPU W, GPU W) plane, and a starvation-aware
allocator picks the best split on that contour every 5 seconds. The EC's stock
temperature-driven fan curve stays untouched as the safety net, so the worst reachable
failure is "fans louder than target" — never "laptop overheats", never "performance
collapses".

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
| `--state-file` | `/var/lib/bazerame-fans/state.json` | Persisted calibration (model + LUT) |
| `--telemetry-dir` | `/var/lib/bazerame-fans/telemetry` | JSONL telemetry logs |
| `--log-dir` | `/var/lib/bazerame-fans/log` | Tracing logs |

Subcommand: `sudo ./target/release/bazerame-fans selftest` — exercises actuators and
sensors end to end (~25 s, no TUI) and prints plain `[ OK ]`/`[FAIL]` lines. Run it once
before trusting the app with a session.

## Configuration

TOML at the `--config` path; every key is optional (a partial file overrides only what it
names, missing/corrupt files fall back to defaults). The fan target and floors are also
editable live from the TUI and written back to this file.

| Key | Default | Meaning |
|---|---|---|
| `fan_target_rpm` | `3000.0` | Steady-state fan RPM Auto mode holds the machine at |
| `cpu_floor_w` | `15.0` | CPU sustained-watts floor: Auto never allocates below this |
| `gpu_floor_mhz` | `1000` | GPU locked-clock floor (MHz): Auto never locks below this |
| `fast_limit_mw` | `53000` | CPU fast (short-burst) PPT limit handed to ryzenadj |
| `online_rls` | `false` | Online RLS slope adaptation in Auto mode. Off by default: field sessions showed the calibrated shape + bounded trim + fan feedback is the robust configuration, while live slope adaptation double-corrects against the trim (and once walked the model into a degenerate contour). Set `true` to experiment |

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
mode and follow the wizard panel:

1. **GPU clock→watts sweep** (10 locked clocks, high to low). Start a saturating
   GPU-heavy load (game or benchmark) when the wizard shows
   `START A GPU-HEAVY LOAD` — the runner verifies the GPU is actually pinned at each
   locked clock before recording.
2. **11-point (CPU W × GPU W) matrix.** The app runs its own in-process CPU burner
   threads; you keep providing (or stopping) the GPU load as prompted. Each point dwells
   until the fans are steady (45 s minimum per point; slow points time out after 4
   minutes).
3. **Fit.** The affine-plus-cross-term thermal model
   (`RPM = a·cpu_W + b·gpu_W + e·cpu_W·gpu_W + c`) is fitted and the max residual
   reported; model + LUT persist to the state file.

Expect roughly 15–40 minutes total, dominated by fan settle times. `Esc` aborts cleanly
at any point.

## Auto mode

Press `a`. Every 5 s the allocator inverts the model to the current fan target's contour
and splits the power budget by per-device starvation; a 1 Hz PI holds GPU watts at its
allocation by moving the locked max clock along the calibrated LUT; the CPU side is
open-loop `ryzenadj` limits, reasserted every 10 s and verified via RAPL. Short CPU
bursts pass through untouched (the fast limit stays at stock). The model's slopes come
from the calibration; the online corrector is a bounded trim integrator (shown dim as
`trim +N rpm`) that absorbs ambient/dust/offset drift. A trust monitor watches the
model's steady-state residuals the whole time — `MODEL DISTRUST` is its
"recalibrate when convenient" hint. Online RLS slope adaptation is off by default
(field-validated as net-destabilizing; see Configuration) and can be enabled with
`online_rls = true` for experimentation, in which case the trust monitor freezes it
whenever the model goes suspect.

Header flags you may see:

| Flag | Meaning |
|---|---|
| `LIMIT-SLIP!` | RAPL keeps measuring above the commanded CPU limit; reasserting |
| `NOT CALIBRATED` | Auto was requested without a calibrated model — run `k` |
| `TARGET UNREACHABLE` | Even the maximum budget cut can't reach the fan target (floors held); check intake/ambient |
| `MODEL DISTRUST` | Model predictions persistently wrong for 5+ min: trim runs at half gain (and RLS, if enabled, is frozen). Treat as a "recalibrate when convenient" hint |
| `THERMAL EMERGENCY` | Tctl ≥ 95 °C or GPU ≥ 87 °C for 3 samples: everything released toward stock. Requires manual re-arm: the first actuating key (`a`, `c`/`g`, `k`) only acknowledges; the second acts |
| `SENSOR LOST` | CPU temperature unreadable for 10 samples while limits were applied: assume hot, same release + re-arm semantics |
| `resumed` | Suspend/resume detected: limits were reasserted, GPU persistence re-enabled, and the limit-slip watchdog runs stricter for 60 s |

## Telemetry

Every run writes one JSONL file under the telemetry dir (falls back to the current
directory if unwritable). Each line carries a `kind` field: one `run_start` header, then
1 Hz `sample` lines (all sensors), `decision` lines (every controller status change or
reassert, with a `cause` string, current limits, flags, and the allocator/model fields on
Auto-mode lines) and `flag` lines (watchdog flag transitions). It loads directly into
pandas (`pd.read_json(path, lines=True)`) or DuckDB (`read_json_auto`) for offline
controller review.

## Safety model

- The EC fan curve is never touched; fans are never commanded. Thermal safety always
  outranks acoustics — every failure path degrades to louder fans or stock behavior,
  never to heat.
- Every exit path (quit, signals, panics, even a controller-thread death) restores stock:
  GPU clock locks reset, CPU limits restored via a platform-profile toggle, `ryzen_smu`
  reloaded, terminal restored. Startup also unconditionally resets to stock, covering a
  previous SIGKILL'd run.
- Watchdogs: thermal emergency and sensor-lost release everything and latch until
  manually re-armed (deliberate two-step, see the flag table); a stickiness check
  verifies via RAPL that CPU limits actually hold; suspend/resume triggers a full
  reassert plus a 60 s strict-checking window.
- Floors bound the controller's authority independently of the adaptive tier.
- `selftest` exercises the whole actuation path before you rely on it.

## File locations

- `/etc/bazerame-fans/config.toml` — fan target, floors, fast limit, `online_rls` (see
  Configuration; written back by the in-app editors; missing/corrupt files fall back to
  defaults, never crash)
- `/var/lib/bazerame-fans/state.json` — calibration state (model + LUT)
- `/var/lib/bazerame-fans/telemetry/` — JSONL telemetry, one file per run
- `/var/lib/bazerame-fans/log/` — tracing logs

## Development

`cargo test` runs the whole suite (no hardware or root needed; hardware-touching tests
are `#[ignore]`d and run manually). The design/decision trail lives in
`docs/plans/` (design + task plan) and `docs/research/` (hardware ground truth, control
theory, stack notes).

Status: code-complete. The on-machine calibration run and a real gaming-session
validation of Auto mode are still pending — everything in the Auto/calibration sections
above describes verified code paths, but end-to-end acoustic performance requires a
calibrated system.

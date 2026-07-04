# bazerame-fans Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this
> plan task-by-task. Progress is tracked in `TODO.md` at the repo root — mark each task's
> checkbox when its final commit lands.

**Goal:** A sudo-run Rust TUI that holds Framework 16 fan noise at/below a user-set RPM
target by shaping sustained CPU/GPU power (design: `docs/plans/2026-07-03-bazerame-fans-design.md`).

**Architecture:** Four OS threads (sampler 1 Hz, controller, input, main/render) joined by
crossbeam channels; TEA-style UI (single Model, `update()`, pure `view()`); controller owns
all hardware writes behind an RAII restore guard. Control = calibrated thermal model
inverted to a fan-RPM contour + allocator + GPU clock PI + bounded trim integrator.

**Tech Stack:** ratatui 0.30 + crossterm, crossbeam-channel, nvml-wrapper, shell-out
ryzenadj, direct sysfs (hwmon/RAPL), pid, nalgebra, serde/toml/serde_json, tracing,
signal-hook, clap, color-eyre.

**Ground rules for every task:**
- TDD: write the failing test first, watch it fail, implement, watch it pass, commit.
- Pure logic (control math, parsing, update()) must be unit-testable without hardware.
  Hardware access goes behind small traits so logic tests use fakes.
- Build as the user (`cargo build`, `cargo test` — never sudo). Manual hardware
  verification runs the built binary: `sudo -S ./target/debug/bazerame-fans < sudo.txt`.
  Password: pipe `sudo.txt` (repo root, gitignored, never commit). Each sudo call stalls
  ~30 s on a fingerprint timeout before reading stdin — this is normal; use generous
  command timeouts (≥120 s).
- Machine facts you may rely on (verified; see design §1): fans at
  `/sys/class/hwmon/hwmon12/fan{1,2}_input` (name `framework_laptop`; discover by name,
  not index), Tctl at hwmon named `k10temp` `temp1_input`, RAPL pkg at
  `/sys/class/powercap/intel-rapl:0/energy_uj` (root-only read), NVML works (RTX 5070),
  `ryzenadj` requires `ryzen_smu` module unloaded, 24 CPU threads.
- Commit after every task (small, descriptive, conventional-commits style).
- Reference research: `docs/research/01-hardware.md` (interfaces), `02-rust.md` (crates),
  `03-control.md` (control math), `04-ratatui.md` (TUI patterns).

---

## Milestone 1 — Monitor (read-only dashboard)

### Task 1: Project scaffold

**Files:** Create `Cargo.toml`, `src/main.rs`, `rustfmt.toml` (empty = defaults).

**Steps:**
1. `cargo init --name bazerame-fans` in repo root.
2. Set `Cargo.toml`:
   ```toml
   [package]
   name = "bazerame-fans"
   version = "0.1.0"
   edition = "2024"

   [dependencies]
   ratatui = "0.30"
   crossterm = "0.29"
   crossbeam-channel = "0.5"
   nvml-wrapper = "0.12"
   pid = "2"
   nalgebra = "0.33"
   serde = { version = "1", features = ["derive"] }
   serde_json = "1"
   toml = "0.8"
   tracing = "0.1"
   tracing-subscriber = "0.3"
   tracing-appender = "0.2"
   signal-hook = "0.3"
   clap = { version = "4", features = ["derive"] }
   color-eyre = "0.6"
   ```
   (If a version fails to resolve, use the nearest available and note it in the commit.)
3. `src/main.rs`: `fn main() { println!("bazerame-fans"); }`
4. Run: `cargo build` → compiles. `cargo run` → prints name.
5. Append `/target` check to `.gitignore` (already has `target/`), commit:
   `feat: scaffold cargo project with dependency stack`

### Task 2: Core types + ring buffer

**Files:** Create `src/types.rs`, `src/ring.rs`; modify `src/main.rs` (mod decls).

**Step 1 — failing tests** (`src/ring.rs` bottom, `#[cfg(test)]`):
```rust
#[test]
fn ring_keeps_last_n() {
    let mut r = Ring::new(3);
    for i in 0..5 { r.push(i as f64); }
    assert_eq!(r.iter().copied().collect::<Vec<_>>(), vec![2.0, 3.0, 4.0]);
    assert_eq!(r.last(), Some(4.0));
}
#[test]
fn ring_handles_empty() {
    let r = Ring::new(3);
    assert_eq!(r.last(), None);
    assert_eq!(r.iter().count(), 0);
}
```
**Step 2:** `cargo test ring` → fails (Ring undefined).
**Step 3:** Implement `Ring` (wrap `VecDeque<f64>`, cap at capacity, `push/last/iter/len`).
Define in `src/types.rs` (no test needed — plain data):
```rust
#[derive(Debug, Clone, Copy, Default)]
pub struct Sample {
    pub t_mono: f64,          // seconds, monotonic
    pub fan1_rpm: f64,
    pub fan2_rpm: f64,
    pub cpu_temp_c: f64,      // Tctl
    pub cpu_pkg_w: f64,       // RAPL delta
    pub igpu_w: f64,          // amdgpu
    pub gpu_w: f64,           // NVML
    pub gpu_temp_c: f64,
    pub gpu_sm_mhz: f64,
    pub gpu_util_pct: f64,
    pub cpu_util_pct: f64,
    pub cpu_avg_mhz: f64,
    pub resumed: bool,        // monotonic jump detected since last sample
}
impl Sample { pub fn max_fan_rpm(&self) -> f64 { self.fan1_rpm.max(self.fan2_rpm) } }
```
**Step 4:** `cargo test` → pass. **Step 5:** commit `feat: add Sample type and ring buffer`.

### Task 3: RAPL power sensor

**Files:** Create `src/sensors/mod.rs`, `src/sensors/rapl.rs`.

Pure logic (wrap-aware delta) is separated from I/O so it's unit-testable.

**Step 1 — failing tests:**
```rust
#[test]
fn watts_from_energy_delta() {
    // 10 J in 2 s = 5 W
    assert_eq!(watts_from_counters(1_000_000, 11_000_000, MAX_RANGE, 2.0), 5.0);
}
#[test]
fn watts_across_wraparound() {
    let max = 1_000_000u64;
    // prev near max, cur small: delta = max - prev + cur = 300_000 uJ over 1s = 0.3 W
    assert!((watts_from_counters(900_000, 200_000, max, 1.0) - 0.3).abs() < 1e-9);
}
#[test]
fn watts_zero_dt_is_zero() {
    assert_eq!(watts_from_counters(0, 100, MAX_RANGE, 0.0), 0.0);
}
```
**Step 3 — implementation:**
```rust
pub fn watts_from_counters(prev_uj: u64, cur_uj: u64, max_range_uj: u64, dt_s: f64) -> f64 {
    if dt_s <= 0.0 { return 0.0; }
    let delta = if cur_uj >= prev_uj { cur_uj - prev_uj } else { max_range_uj - prev_uj + cur_uj };
    delta as f64 / 1e6 / dt_s
}
```
Plus `RaplReader` struct: opens `/sys/class/powercap/intel-rapl:0/{energy_uj,max_energy_range_uj}`,
`read_watts(&mut self) -> Option<f64>` keeping prev counter+timestamp internally.
Constructor takes the base path (`&Path`) so tests can point at a tempdir fixture —
add one test writing fixture files to `std::env::temp_dir()` subdir and reading twice.
**Step 5:** commit `feat: RAPL package power sensor with wraparound handling`.

### Task 4: hwmon sensors (fans, Tctl, amdgpu)

**Files:** Create `src/sensors/hwmon.rs`.

**Behavior:** scan `<root>/hwmon*/name`; build map name→dir. Expose:
- `Hwmon::discover(root: &Path) -> Self`
- `fan_rpms() -> (f64, f64)` from the `framework_laptop` (fallback `cros_ec`) dir's
  `fan1_input`/`fan2_input`
- `cpu_temp_c() -> f64` from `k10temp` `temp1_input` (millidegrees / 1000)
- `igpu_w() -> f64` from `amdgpu` `power1_average` (microwatts / 1e6)
Missing files → 0.0 (sensor absence must never crash the sampler); log at debug.

**Step 1 — failing test:** build a fixture tree in a tempdir
(`hwmon0/name`=`k10temp`, `hwmon0/temp1_input`=`49375`; `hwmon1/name`=`framework_laptop`,
`fan1_input`=`1467`, `fan2_input`=`1452`; `hwmon2/name`=`amdgpu`, `power1_average`=`8041000`),
assert `discover` finds all three and values convert correctly (49.375 °C, 1467/1452 RPM, 8.041 W).
**Step 5:** commit `feat: hwmon discovery and fan/temp/igpu sensors`.

### Task 5: CPU utilization + frequency sensors

**Files:** Create `src/sensors/cpu.rs`.

- Util: parse `/proc/stat` first line; util% = 1 − Δidle/Δtotal (keep prev counters).
  Pure function `util_from_stat_lines(prev: &str, cur: &str) -> f64` — unit-test with two
  literal `cpu  ...` lines with known deltas (include iowait in idle).
- Freq: mean of `/sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq` (kHz → MHz).
  Path-injectable like hwmon; fixture test with two fake cpus.

Commit: `feat: cpu utilization and average frequency sensors`.

### Task 6: NVML sensor wrapper

**Files:** Create `src/sensors/gpu.rs`.

Thin wrapper — no unit tests (it's all FFI); verified manually in Task 9.
```rust
pub struct GpuSensor { nvml: Nvml, /* device by index 0 */ }
impl GpuSensor {
    pub fn new() -> color_eyre::Result<Self>;
    pub fn read(&self) -> GpuReading { /* power_usage()/1000, temperature(), clock_info(Sm), utilization_rates().gpu */ }
}
```
All getters individually `unwrap_or(0)`-style tolerant (return Option → 0.0), so a driver
hiccup never kills sampling. Commit: `feat: NVML GPU sensor wrapper`.

### Task 7: Sampler thread + event enum

**Files:** Create `src/event.rs`, `src/sensors/sampler.rs`.

```rust
pub enum Event {
    Sample(Sample),
    Input(crossterm::event::KeyEvent),
    Status(ControlStatus),   // added Task 14; stub the variant now with an empty struct
    Tick,
}
```
Sampler: owns all sensor structs; loop `{ read all → Sample; send to ui_tx and ctl_tx;
sleep to next 1 s boundary }`. Resume detection: pure function tested first:
```rust
#[test]
fn detects_monotonic_gap() {
    assert!(!is_resume_gap(1.0, 2.0));   // normal 1 s cadence
    assert!(is_resume_gap(1.0, 9.0));    // > 5 s gap ⇒ we slept
}
```
(`is_resume_gap(prev_t, cur_t) = cur_t - prev_t > 5.0`; sets `sample.resumed`).
Thread exits when a shared `Arc<AtomicBool>` shutdown flag flips.
Commit: `feat: 1Hz sampler thread with resume detection`.

### Task 7b: Telemetry JSONL logger

**Files:** Create `src/telemetry.rs`.

Purpose: offline controller-quality review — every sample and every controller decision
land in an append-only JSONL file loadable into pandas/DuckDB.

```rust
#[derive(serde::Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Record<'a> {
    Sample(&'a Sample),
    Decision(&'a DecisionRecord),   // defined in Task 14; stub the variant until then
    Flag { t_mono: f64, flag: String, active: bool },
}
pub struct Telemetry { /* BufWriter<File> */ }
impl Telemetry {
    /// path e.g. /var/lib/bazerame-fans/telemetry/run-<unix_ts>.jsonl (dir created);
    /// CLI-overridable; falls back to ./telemetry-<ts>.jsonl if dir unwritable
    pub fn open(dir: &Path) -> std::io::Result<Self>;
    pub fn log(&mut self, r: &Record) ;   // serialize + \n; flush every 10 records or 5 s
}
```
Failure to open/write telemetry must never crash sampling — log a tracing warning once
and drop records. Tests: roundtrip (open in tempdir, log 3 records incl. a Sample,
read file back, each line parses as JSON with correct `kind`); unwritable dir → open
falls back / errors without panic. Wire into main (Task 9): sampler samples are logged
by the main loop on receipt. Controller decisions wired in Task 14+.
Commit: `feat: JSONL telemetry log for offline controller analysis`.

**Downstream requirements this creates:** Task 14 defines `DecisionRecord` (mode,
setpoints commanded, why: demand scores, contour budget, trim offset, PI internals,
flags) and logs one per control step; Tasks 25/26/27 extend it as allocator/trim/RLS
land. Task 22 logs calibration points as they're recorded. Acceptance for M4/M5
includes reviewing a session's telemetry offline.

### Task 8: UI Model + update()

**Files:** Create `src/model.rs`.

`Model` holds: `Ring` buffers (capacity 300 = 5 min) for max_fan/cpu_w/gpu_w/cpu_temp/
gpu_temp/gpu_mhz, latest `Sample`, latest `ControlStatus`, `running: bool`, and
`fan_target_rpm: f64` (display only for now).

**Step 1 — failing tests:**
```rust
#[test]
fn sample_event_fills_rings() {
    let mut m = Model::new();
    m.update(Event::Sample(Sample { fan1_rpm: 2000.0, ..Default::default() }));
    assert_eq!(m.max_fan.last(), Some(2000.0));
}
#[test]
fn q_key_quits() {
    let mut m = Model::new();
    m.update(Event::Input(key('q')));
    assert!(!m.running);
}
```
(`key()` helper constructs a `KeyEvent`.) Keymap now: `q` quit only — more in M2.
Commit: `feat: TEA model and update loop`.

### Task 9: Dashboard view + main wiring (runnable milestone)

**Files:** Create `src/ui/mod.rs`, `src/ui/view.rs`, `src/logging.rs`; rewrite `src/main.rs`.

**view(model, frame):** layout: header bar (title, target, status flags) /
2×2 chart grid (fans RPM [+target line], watts CPU+GPU overlaid, temps, GPU clock) /
footer keybar. Charts: ratatui `Chart` with `Dataset` per series windowed from rings.
**Step 1 — TestBackend smoke test:**
```rust
#[test]
fn view_renders_without_panic_on_empty_and_full_model() {
    let mut t = Terminal::new(TestBackend::new(120, 40)).unwrap();
    t.draw(|f| view(&Model::new(), f)).unwrap();
    let mut m = Model::new();
    for i in 0..400 { m.update(Event::Sample(sample_with(i as f64))); }
    t.draw(|f| view(&m, f)).unwrap();
}
```
**main.rs:** color_eyre install → logging init (tracing-appender →
`/var/lib/bazerame-fans/log/` if writable else `./bazerame-fans.log`) → require root
(`nix`-free check: `std::fs::metadata("/proc/self").uid() == 0` or read euid via
`unsafe { libc::geteuid() }` — simplest: attempt opening RAPL energy_uj, friendly error) →
channels, spawn sampler + input threads → `ratatui::init()` → loop
`{ draw; select! on rx with 100ms tick }` → on quit: flip shutdown flag,
`ratatui::restore()`.
**Manual verification:** `cargo build && sudo -S ./target/debug/bazerame-fans < sudo.txt`
in a real terminal — confirm live fan RPM/watts/temps update each second; `q` exits and
terminal is intact. (Agent: run it with a 15 s timeout piping `q` after; human does the
visual check.)
Commit: `feat: live monitoring dashboard (milestone 1)`.
Mark M1 complete in TODO.md.

---

## Milestone 2 — Manual actuation + safety plumbing

### Task 10: Command runner trait + ryzen_smu module handling

**Files:** Create `src/actuators/mod.rs`, `src/actuators/cmd.rs`, `src/actuators/smu_module.rs`.

`cmd.rs`: `pub trait Runner { fn run(&self, program: &str, args: &[&str]) -> Result<Output>; }`
with `RealRunner` (std::process::Command) and a test `FakeRunner` recording invocations
and returning scripted results.

`smu_module.rs` logic (unit-test with FakeRunner + fixture paths):
- `needs_unload(sysfs: &Path) -> bool`: true iff `<sysfs>/ryzen_smu_drv` exists AND
  `<sysfs>/ryzen_smu_drv/pm_table` does NOT exist (the broken-for-Strix state).
- `ensure_unloaded(runner)` → runs `modprobe -r ryzen_smu` when needed; remembers it did.
- `restore(runner)` → `modprobe ryzen_smu` only if we unloaded it.

Tests: fixture dir with/without `pm_table`; assert FakeRunner saw exactly the expected
modprobe calls, and restore is a no-op when nothing was unloaded.
Commit: `feat: ryzen_smu module detection and unload/restore`.

### Task 11: CPU actuator (ryzenadj) + stock restore

**Files:** Create `src/actuators/cpu.rs`.

```rust
pub struct CpuActuator<R: Runner> { runner: R, pub fast_limit_mw: u32 /* stock burst, default 53000 */ }
impl CpuActuator {
    /// clamp to [10_000, 54_000] mW then: ryzenadj --stapm-limit=X --slow-limit=X --fast-limit=<fast>
    pub fn set_sustained_mw(&self, mw: u32) -> Result<()>;
    /// restore stock: write platform_profile to "low-power" then back to its prior value
    pub fn restore_stock(&self, profile_path: &Path) -> Result<()>;
}
```
Tests (FakeRunner + tempdir profile file containing `balanced`):
- `set_sustained_mw(20_000)` → exactly one ryzenadj call with
  `["--stapm-limit=20000","--slow-limit=20000","--fast-limit=53000"]`.
- clamping: `set_sustained_mw(5_000)` → args contain `--stapm-limit=10000`.
- `restore_stock` → profile file ends containing `balanced` again (write low-power, then
  original; use real file I/O in tempdir).
Commit: `feat: CPU sustained power actuator with clamps and stock restore`.

### Task 12: GPU actuator (NVML clock locks)

**Files:** Create `src/actuators/gpu.rs`.

```rust
pub struct GpuActuator { /* nvml device */ }
impl GpuActuator {
    pub fn new() -> Result<Self>;                      // also enables persistence mode (ignore failure, log)
    pub fn set_max_clock(&mut self, mhz: u32) -> Result<()>;  // set_gpu_locked_clocks(210, clamp(mhz, 1000..=3090))
    pub fn release(&mut self) -> Result<()>;           // reset_gpu_locked_clocks()
    pub fn applied(&self) -> Option<u32>;
}
```
Pure clamp logic factored to `fn clamp_gpu_clock(mhz: u32) -> u32` — unit-test bounds
(999→1000, 5000→3090, 1500→1500). NVML calls themselves are verified manually in Task 16.
Commit: `feat: GPU max-clock actuator via NVML locked clocks`.

### Task 13: Restore guard + startup reset

**Files:** Create `src/actuators/guard.rs`.

`RestoreGuard` owns `CpuActuator + GpuActuator + SmuModule state`; `Drop` runs:
GPU `release()` → CPU `restore_stock()` → smu `restore()`, each failure logged not
propagated. Also `pub fn startup_reset(...)` running the same sequence at boot.
Test: with fakes, dropping the guard produces the full restore sequence in order.
Wire panic hook in main to also fire hardware restore (hook holds a channel sender or
the guard lives in controller thread and main joins it before `ratatui::restore` — keep
it simple: controller thread owns guard; main signals shutdown, joins controller with
timeout, then restores terminal).
Commit: `feat: RAII hardware restore guard and startup reset`.

### Task 14: Controller thread + ControlStatus

**Files:** Create `src/control/mod.rs`, `src/control/controller.rs`; extend `src/event.rs`.

```rust
pub enum Command { SetCpuW(f64), SetGpuMaxClock(u32), ReleaseAll, Pause, Resume,
                   SetFanTarget(f64), SetFloors { cpu_w: f64, gpu_mhz: u32 }, Quit }
#[derive(Clone, Debug, Default)]
pub struct ControlStatus {
    pub mode: Mode,                    // Monitor | Manual | Auto(later) | Calibrating(later)
    pub cpu_limit_w: Option<f64>, pub gpu_max_mhz: Option<u32>,
    pub flags: Vec<StatusFlag>,        // e.g. LimitNotSticking, Resumed
}
```
Also define `DecisionRecord` (serializable; consumed by Task 7b telemetry): t_mono, mode,
commanded cpu_w / gpu_max_mhz, and a `why` payload (later tasks extend it: demand scores,
contour budget, trim offset, PI internals). Controller logs one Decision per control
action and a Flag record on every status-flag transition.

Controller loop (`select!` on samples/commands/10 s reassert tick):
- Manual commands → actuators; every status change → `Event::Status` to UI.
- Reassert tick: reapply current CPU limit (defends against PPD/tuned clobbers).
- Stickiness check: if `cpu_limit` set and 3 consecutive samples show
  `cpu_pkg_w > limit + 5.0` → reapply + flag `LimitNotSticking`.
- `Sample.resumed` → reapply everything, flag `Resumed` (clears after 30 s).
Logic is testable: extract `fn on_sample(&mut self, s: &Sample) -> Vec<Action>` pure-ish
core with fakes; tests: stickiness triggers after exactly 3 bad samples; resume triggers
reapply.
Commit: `feat: controller thread with manual mode, reassert and stickiness watchdog`.

### Task 15: Manual-mode UI

**Files:** Modify `src/model.rs`, `src/ui/view.rs`.

Keys: `c/C` CPU limit −/+ 2 W (range 10–54, `x` = release CPU), `g/G` GPU max clock
−/+ 105 MHz (≈14 bins; range 1000–3090, `X` = release GPU), `p` pause/release-all,
`q` quit. Status panel shows applied limits + flags from `ControlStatus`.
Tests in `model.rs`: key `c` emits `Command::SetCpuW(next_value)` (Model returns
`Vec<Command>` from `update`, main forwards to controller); TestBackend render with
status set.
Commit: `feat: manual actuation controls in TUI`.

### Task 16: Milestone 2 end-to-end verification

No new files. Build; run
`sudo -S ./target/debug/bazerame-fans < sudo.txt` alongside a CPU load
(the agent may run 24 shell spinners for 60 s). Script the TUI? No — verify at the
actuator level with a disposable `--selftest` hidden subcommand instead:
`bazerame-fans selftest` runs: startup_reset → set CPU 20 W → spawn built-in 10 s burn →
read RAPL (expect ≤ 22 W) → set GPU max 1200 MHz → read NVML lock state → release all →
print PASS/FAIL lines and exit. Implement `selftest.rs` (this doubles as the future
calibration burner seed). Run it, capture output, fix until PASS.
Commit: `feat: hardware selftest subcommand (milestone 2 verified)`.
Mark M2 complete in TODO.md.

---

## Milestone 3 — Calibration

### Task 17: CPU burner

**Files:** Create `src/calib/mod.rs`, `src/calib/burner.rs`.

`Burner::start(n_threads)` spawns spin loops (`std::hint::black_box` on a counter),
`stop()` joins via AtomicBool. Test: start(2) → threads run → stop() returns promptly
(< 1 s). (Already partially exists from selftest — refactor selftest to use this.)
Commit: `feat: in-process CPU burner`.

### Task 18: Steady-state detector + point recorder

**Files:** Create `src/calib/steady.rs`.

Pure functions over sample windows:
- `is_steady(rpm_window: &[f64]) -> bool` — max−min of last 20 samples < 100 RPM.
- `point_value(window: &[f64]) -> f64` — mean of last 20.
Tests: flat window → steady; ramping window → not; mean computed over exactly the tail.
Commit: `feat: steady-state detection for calibration`.

### Task 19: Clock→watts LUT sweep

**Files:** Create `src/calib/lut_sweep.rs`, `src/control/lut.rs`.

`lut.rs`: `ClockWattsLut(Vec<(u32 /*mhz*/, f64 /*watts*/)>)` with
`clock_for_watts(w) -> u32` (linear interpolation, clamped ends; input non-monotonic
tolerated by sorting on insert). Tests: known 3-point LUT interpolates and clamps.

`lut_sweep.rs`: state machine driven by `on_sample`, not wall-clock sleeps (testable):
states `WaitPinned → Settle(clock) → Record → next clock`; clocks swept 3090→1200 in
~10 steps. `WaitPinned` requires `gpu_util > 90 && |sm_mhz − lock| < 30` else emits
`NeedsLoad` status (UI shows "start a GPU-heavy load now"). Feed synthetic samples in
tests: full sweep produces a 10-entry LUT; unpinned samples stall in WaitPinned.
Commit: `feat: GPU clock-to-watts calibration sweep`.

### Task 20: Thermal model fit + RLS

**Files:** Create `src/control/thermal_model.rs`.

Model: `rpm = a·pc + b·pg + e·pc·pg + c` (design §3).
- `fit_batch(points: &[(f64 /*pc*/, f64 /*pg*/, f64 /*rpm*/)]) -> ThermalModel` —
  least squares via nalgebra (4-col design matrix, `svd.solve`).
- `predict(pc, pg) -> f64`; `residuals(points) -> Vec<f64>`.
- `rls_update(&mut self, pc, pg, rpm, lambda: f64)` — 4×4 covariance RLS with
  forgetting; skip update (return false) if it would make `a` or `b` negative.
- `budget_contour(&self, target_rpm, trim_offset) -> impl Fn(pc: f64) -> f64` —
  solve for pg given pc: `pg = (target − c − trim − a·pc) / (b + e·pc)` clamped ≥ 0.

**Tests (all synthetic, deterministic):**
- generate 11 points from known params (a=25, b=15, e=0.1, c=800) → fit recovers within
  1e-6; residuals ~0.
- add noise ±20 RPM → params within 10%.
- `rls_update` on drifted offset converges c toward new value; slope-sign guard rejects
  poisoned update.
- contour: `predict(pc, contour(pc)) ≈ target` for pc ∈ {10, 25, 40}.
Commit: `feat: affine+cross thermal model with batch fit and gated RLS`.

### Task 21: Config + state persistence

**Files:** Create `src/state.rs`, `src/config.rs`.

`config.rs`: `Config { fan_target_rpm: f64 (default 3000), cpu_floor_w: f64 (15),
gpu_floor_mhz: u32 (1000), fast_limit_mw: u32 (53000) }` — load
`/etc/bazerame-fans/config.toml` if present else defaults; `--config` CLI override.
`state.rs`: `PersistedState { model: Option<ThermalModel>, lut: Option<ClockWattsLut>,
calibrated_at: Option<String> }` — load/save `/var/lib/bazerame-fans/state.json`
(CLI-overridable path), atomic save (write `.tmp`, rename). Tests: roundtrip in tempdir;
missing file → default; corrupt JSON → default + logged (never crash).
Commit: `feat: config loading and atomic state persistence`.

### Task 22: Calibration runner + UI wizard

**Files:** Create `src/calib/runner.rs`; modify `src/control/controller.rs`,
`src/model.rs`, `src/ui/view.rs`.

Runner = state machine composing Tasks 17–21: LUT sweep → 11-point matrix
(design §4 table; CPU via burner + ryzenadj, GPU via LUT-commanded clock, both via
actuators) → batch fit → residual report → persist. Emits `CalibProgress` status
(step i/N, current point, settle countdown, NeedsLoad prompt, final residual).
Driven by `on_sample` (testable with synthetic sample streams — full run completes and
produces a state file in a tempdir; abort command mid-run restores and returns to
Monitor mode). UI: `k` starts calibration; wizard panel shows progress/prompts;
`Esc` aborts. Manual: run a real calibration (~30 min, user present for GPU load) —
this is a *human-gated* step; agent implements + unit-tests, human runs later.
Commit: `feat: guided calibration mode (milestone 3)`.
Mark M3 complete in TODO.md.

---

## Milestone 4 — Closed loop

### Task 23: Demand estimator + allocator

**Files:** Create `src/control/allocator.rs`.

```rust
pub struct Demand { pub cpu_starved: f64, pub gpu_starved: f64 } // 0..1 each
pub fn demand(s: &Sample, cpu_limit_w: f64, gpu_target_w: f64) -> Demand
// starvation = min(1, draw/allowed) with pinned-at-limit boost (design §3, research 03 §4)
pub struct Allocator { /* rate limits, deadband, last split */ }
impl Allocator {
    /// choose (cpu_w, gpu_w) on the contour honoring floors; asymmetric rate limits:
    /// down fast (≤8 W/step), up slow (≤2 W/step); deadband: no change if measured
    /// max_fan within ±150 RPM of target and demand split unchanged materially
    pub fn step(&mut self, contour: &dyn Fn(f64) -> f64, demand: Demand,
                floors: (f64, f64), total_prev: (f64, f64)) -> (f64, f64);
}
```
Tests (synthetic): GPU-starved workload shifts watts to GPU along contour; both-starved
holds proportional split; floors never violated; asymmetry (large downward step allowed,
upward capped at 2 W); deadband produces zero change.
Commit: `feat: demand-weighted contour allocator with asymmetric rate limits`.

### Task 24: GPU watts→clock inner PI

**Files:** Create `src/control/gpu_pid.rs`.

Wrap `pid` crate: setpoint = allocated GPU watts, measurement = NVML watts, output =
clock *offset* from LUT feedforward: `clock = lut.clock_for_watts(target) + pi_out`,
clamped, deadband ±3 W (inside deadband → no clock change), rate limit 105 MHz/step,
anti-windup via pid crate's limits. Simulated-plant test: fake GPU where
`watts = 0.04·mhz − 20` + noise; loop 60 iterations → converges within ±3 W of target;
no oscillation > 1 bin after convergence.
Commit: `feat: GPU watts-to-clock PI with LUT feedforward`.

### Task 25: Auto mode wiring + UI

**Files:** Modify `src/control/controller.rs`, `src/model.rs`, `src/ui/view.rs`.

Controller `Mode::Auto`: every 5 s tick → demand + allocator step → CPU actuator set +
GPU PI setpoint; every 1 s → GPU PI update. Requires calibrated state (else flag
`NotCalibrated`, refuse Auto). UI: `a` toggles Auto, `t/T` fan target ±250 RPM
(persisted to config on change), target line on fan chart, mode + allocations in status
panel. Tests: controller state transitions (Monitor→Auto requires model; Pause releases
limits but keeps mode memory); model.rs key tests.
Commit: `feat: closed-loop auto mode against fan target (milestone 4)`.
Mark M4 complete in TODO.md. Human validation: run a real game session.

---

## Milestone 5 — Adaptive & hardening

### Task 26: Bounded trim integrator

**Files:** Create `src/control/trim.rs`.

`Trim { offset_rpm: f64 }`; every 20 s, if steady (Task 18 detector) :
`offset += ki · (measured_rpm − predicted_rpm)`, ki ≈ 0.05, saturated so that induced
budget cut ≤ 25% (compute via model sensitivity) and floors always win. Tests: sustained
+300 RPM error walks offset up to cap and stops; error sign flip walks it back;
non-steady samples produce no update.
Commit: `feat: bounded ambient trim integrator`.

### Task 27: Online RLS + trust monitor

**Files:** Modify `src/control/thermal_model.rs`; create `src/control/trust.rs`.

Wire `rls_update` (λ=0.99) into controller on steady samples in Auto mode. Trust monitor:
EWMA of |residual|; > 300 RPM sustained 5 min → freeze RLS + halve trim gain + flag
`ModelDistrust`. Tests: injected model mismatch trips distrust at expected time;
recovery clears it.
Commit: `feat: online model adaptation with trust monitor`.

### Task 28: Watchdogs + emergency release

**Files:** Create `src/control/watchdog.rs`; modify controller.

Rules (design §3): Tctl ≥ 95 °C or GPU ≥ 87 °C for 3 consecutive samples → release all
caps, flag `ThermalEmergency`, require manual re-arm (`a`). Tests: synthetic hot samples
trigger exactly once; re-arm resets.
Commit: `feat: thermal emergency watchdog`.

### Task 29: Resume/clobber hardening + polish pass

**Files:** Modify controller (resume reassert exists — extend to GPU + verify), `src/ui/`.

- On `Resumed`: full reassert (CPU + GPU + persistence mode) and 60 s of elevated
  stickiness checking.
- Floors editable in UI (`f` cycles focus, arrows adjust) and persisted.
- Status flags surfaced with colors; log file line on every flag transition.
- `cargo clippy -- -D warnings` clean; README.md with usage (sudo invocation, keys,
  calibration walkthrough, Bazzite ryzen_smu note).
Commit: `feat: resume hardening, floor editing, docs (milestone 5)`.
Mark M5 complete in TODO.md. Human: 30-min gaming acceptance test + abuse tests
(design §7 acceptance criteria).

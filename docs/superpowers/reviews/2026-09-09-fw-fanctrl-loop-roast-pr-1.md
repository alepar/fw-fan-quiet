---
super-roast verdict: Blocking (21 confirmed)
mode: PR        iteration: 1 of 3
profile (assumed): Single-operator personal tooling with hardware side-effects: a Rust daemon on one Framework 16 laptop that writes CPU power limits via ryzenadj (sudo) and dGPU clock locks via NVML, and reads the EC through cros_ec/ectool. No network surface, no other users, no external data. Blast radius is the operator's own machine: a wrong cap or a stuck emergency release degrades performance or acoustics and, at worst, lets the hardware's own thermal protection (card slowdown 89 C / shutdown 92 C; EC trip points) take over — the hardware backstops exist and are documented in the design. Rollback is a git revert and a daemon restart. Thermal-safety inversions, silent loss of control (NaN/latched states), and anything that defeats the hardware backstops are the material class; resilience/observability polish is low-value here.
inputs: epic-fw-fanctrl-loop-6ma-integration@3bf65d8 vs master@10a730f
coverage: triage live, 12 scouts ran (0 dead), dedupe live · 102 raw → 70 deduped → 29 panel / 41 spot-checked · judge completion 100% · remainder-capped: 0
independence: same-family (Claude) — seat-differentiated panel
seat-agreement: panels 29 · rr 0.72 · rg 0.76 · fg 0.69 · unanimous 0.59 · ground-loo 0.81 (n=21) · reproduce 18/11/0 · refute 16/13/0 · ground 25/4/0

## Confirmed findings

- [Blocking] src/sensors/poller.rs:69-78; src/sensors/sampler.rs:237 — `FanctrlPoller::tick` holds the shared `Arc<Mutex<Box<dyn FanctrlSource>>>` across the entire blocking socket round trip, so the sampler's 1 Hz `merge_fanctrl` lock blocks for the full poll — contradicting the module doc's own "can never stall a Sample" invariant and stalling the stream that feeds the PI loop, the thermal watchdog and the actuator reassert.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓)
  evidence: poller.rs:69-71 `let mut src = self.source.lock()…; src.poll(PrintCommand::All, now); drop(src)` (same for Speed); client.rs:391-411 `send` does connect (1 s) + write + `read_to_string` (3 s per read) under that guard; sampler.rs:237 locks the identical `SharedFanctrl` every tick; poller.rs:6-11 asserts the opposite. Only one mutex, no double-buffer; the NVMe path got its own thread + test (sampler.rs:579) for exactly this class, fanctrl did not. RESUME_GAP_S = 5.0 (sampler.rs:71) makes a ~4 s hold able to spoof `resumed`. Unanimous Blocking; the material class here is the watchdog going blind while caps stay applied. (Refute seat correction: the two polls are separate ~4 s holds, not one 8 s hold.)
  fix-shape hint: poll outside the lock — take a snapshot/clone of the client (or move the client out of the mutex and only publish the resulting `FanctrlView`/`Freshness` into a small `Mutex<Snapshot>`), so `merge_fanctrl` only ever locks a value that is never held across I/O; add the sampler-not-stalled-by-hung-fanctrl test mirroring the NVMe one.

- [Should-fix] src/fanctrl/client.rs:398-411 — `UnixFanctrlClient::send` has no overall deadline and no size cap: `SO_RCVTIMEO` is per-read, `read_to_string` loops to EOF, so a trickling peer or oversized `print all` body keeps one `poll` — and the fanctrl mutex — held indefinitely.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓)
  evidence: client.rs:396-411 sets `set_read_timeout` then `read_to_string(&mut body)` with no `Read::take` or elapsed check; grounded against https://doc.rust-lang.org/std/os/unix/net/struct.UnixStream.html#method.set_read_timeout and https://man7.org/linux/man-pages/man7/socket.7.html ("timeout applies to each individual system call"). Combined with the finding above the sampler blocks forever on sampler.rs:237. Not a documented tradeoff (§2.1 promises 1 s/3 s).
  fix-shape hint: wrap the stream in `Read::take(MAX_BODY)` and check a total `Instant` deadline inside the read loop (or read with `BufReader` until EOF/deadline); fold into the same change as the lock fix.

- [Should-fix] src/sensors/sampler.rs:237; src/sensors/poller.rs:118 — The sampler fail-stops on a poisoned fanctrl/NVMe mutex (`.expect(...)`), unlike the deliberately poison-tolerant telemetry lock, so a panic in either poller thread while holding its lock kills the sample stream while the controller keeps applied caps and the watchdog stops observing — and the process does not exit.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓)
  evidence: sampler.rs:237, poller.rs:69,75,118,151 use `.lock().expect(...)`; telemetry.rs:288-297 uses `unwrap_or_else(PoisonError::into_inner)` with the comment "keep logging instead of cascading the panic" (pre-existing convention on master). Reachable poison: client.rs:334 `thread::spawn` panics on OS thread-creation failure (https://doc.rust-lang.org/std/thread/fn.spawn.html "Panics if the OS fails to create a thread") while `tick` holds the lock. controller.rs:2725-2731 sets `sample_rx = never()` on disconnect and keeps running; `watchdog.observe` (controller.rs:1178) is only called from `on_sample`. Both lock sites are new in this PR.
  fix-shape hint: reuse the telemetry `lock()` helper (poison-tolerant) at all five sites, and/or use `thread::Builder::spawn` in `connect_with_timeout` and map the error to `FanctrlError::Other`.

- [Should-fix] src/actuators/cpu.rs:138; src/control/controller.rs:1941-1946; src/actuators/cmd.rs:23-25; src/main.rs:325-334 — Read-back verification makes every CPU limit write two (four on mismatch) `ryzenadj` `Command::output()` calls with no timeout, inline on the controller thread that owns the watchdog reaction, `Quit` handling and stock restore; `ctl.join()` has no timeout and runs before `ratatui::try_restore()`.
  verdict: confirmed (reproduce ✓ / refute ✗ (rejected: pre-existing) / ground ✓)
  evidence: cpu.rs:106-139 `set_sustained_mw` → `verify_write` → `runner.run("ryzenadj", ["--info"])`; controller.rs:1938-1945 re-invokes the whole write+verify on Mismatch (comment says "re-read once"); cmd.rs:23-25 `Command::output()` blocking, no timeout; controller.rs:2711-2714 comment asserts "Nothing here may block long"; main.rs joins before terminal restore. Ground seat grounded the hang possibility in RyzenAdj's own `lib/nb_smu_ops.c` unbounded `while(response == 0x0)` spin (https://github.com/FlyGoat/RyzenAdj/blob/master/lib/nb_smu_ops.c). Refute seat is right that the single-call exposure pre-exists on master (`git show master:src/actuators/cpu.rs`); the branch doubles/quadruples the calls per tick, and the §2.9 read-back itself is new, so it stands as worsened, not novel.
  fix-shape hint: give `Runner::run` a `timeout` (spawn + `wait_timeout` + kill), or move the `--info` read-back off the controller thread and feed the verdict back as a Sample-side field; bound `ctl.join()` with a fallback restore.

- [Should-fix] src/control/controller.rs:1849; src/control/controller.rs:1560 — `f64::signum()` returns `1.0` for `+0.0`, so both `error_sign` derivations report "calling for more budget" when the error is exactly zero or not computable, arming the demand-limited integrator halt and the §2.7 `TARGET UNREACHABLE (high)` rule in a case the code intends to be neutral.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓)
  evidence: all three seats compiled `0.0f64.signum()` → `1` with the repo's rustc; controller.rs:1849 `err_opt.map(loop_error_value).unwrap_or(0.0).signum()` (the `unwrap_or(0.0)` shows the intended neutral); `compute_loop_error` (1706-1727) returns `None` for RpmLoop with non-finite `rpm_smoothed`; budget.rs:313-321 documents "error_sign <= 0.0 … is never halted here"; mode.rs:419 `error_sign > 0.0` → high-unreachable. `AutoState::new` initialises `last_error_sign: 0.0` un-signum'd. Ground seat notes 1560 sits inside `if let Some(err)` so only the exact-zero half applies there.
  fix-shape hint: a small `sign3(x) -> f64` returning 0.0 for `x == 0.0`/NaN (or keep `Option<f64>` and treat `None` as neutral) used at both sites.

- [Should-fix] src/control/mode.rs:58,97-101; src/control/controller.rs:1483-1528 — `Arbiter::decide`'s tick-counted hysteresis constants are documented against the 5 s allocator cadence but `decide` runs on every 1 Hz sample, so entry hysteresis and the feasible-again clear run 5x faster than spec (15 s → 3 s, 60 s → 12 s).
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓)
  evidence: mode.rs:9-14 "called once per allocator tick (5 s)"; mode.rs:96-101 `FEASIBLE_CLEAR_TICKS = 12` derived as 60 s / PI_PERIOD_S; controller.rs:1483-1486 "Every sample: the arbiter tick" outside the `if due` gate; commit a66971e deliberately moved decide to 1 Hz for §2.6 reconciliation but never rescaled the constants; controller.rs:5251-5253/5282-5285 tests already observe TempLoop engaging on the 3rd 1 Hz sample. Design doc §2.5 line 547 "3 consecutive ticks (15 s)"; fwloop.10 acceptance "60 s of continuous feasibility". Refute seat correction: the argmax debounce is specified in samples (design doc line 543), so that third counter is correct at 1 Hz — only entry hysteresis and feasible-clear are wrong.
  fix-shape hint: either gate the streak counters on `due` (keep decide at 1 Hz for reconciliation) or rescale `ENTRY_HYSTERESIS_TICKS`/`FEASIBLE_CLEAR_TICKS` by the 1 Hz cadence and fix the mode.rs module doc; add a wall-clock test for both timers.

- [Should-fix] src/state.rs:65; src/fanctrl/table.rs:113,157; src/control/budget.rs:381,392 — `PersistedState::load` deserialises `duty_rpm_table` and `loop_gains` with no invariant checking: an empty `points` map panics `duty_for_rpm`, a non-monotone table panics `refine` inside `f64::clamp`, and `ti_s`/`ti_rpm_s` of 0 makes `Budget::step` divide by zero and drive `u` permanently to NaN — the calibration path validates, the load path does not.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓)
  evidence: state.rs:60-67 only rejects parse errors; table.rs:113 `best.expect(...)`; table.rs:157 `blended.clamp(lo+margin, hi-margin)` — seats verified `f64::clamp` panics on `min > max` (compiled locally; https://doc.rust-lang.org/src/core/num/f64.rs.html `const_assert!(min <= max)`); `is_strictly_increasing` is `#[cfg(test)]` only; budget.rs:381/392 divide by `ti` straight from persisted gains via `Budget::new` (budget.rs:219); `derive_one` (fopdt.rs:236-249) validates only on the calibration path. state.rs module doc promises "Loading NEVER crashes". NaN-driving-u is the profile's named material class; kept Should-fix rather than Blocking because the writer is this daemon itself (root-owned file, operator edit or schema skew required).
  fix-shape hint: a `PersistedState::validated()` step after load that falls back to defaults (with a warn) on empty/non-monotone table or non-finite/non-positive gains; make `is_strictly_increasing` a runtime check.

- [Should-fix] src/config.rs:164-218; src/control/guards.rs:105-112 — `gpu_hot_c`/`nvme_hot_c` are the only numeric `Config` fields not passed through `Config::sanitized`, so NaN silently disables the dGPU guard, a value at/below idle latches it permanently hot (ratcheting the GPU share to its floor forever), and a value above `GPU_TRIP_C` lets the hard watchdog fire first — with no clamp-and-warn and no log.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓)
  evidence: config.rs `sanitized()` clamps `gpu_floor_mhz/cpu_max_w/gpu_max_w/cpu_floor_w` each with `is_finite` + `tracing::warn!` and never touches the two new keys (added by this PR); controller.rs:1073 → `Guards::new` (674) → guards.rs `hysteresis` `if !was_hot && t >= enter_c … else if was_hot && t <= enter_c - band_c` — NaN comparisons are all false. controller.rs:652-661 records this branch already fixed the sibling "value never reached the guard" defect. Seat correction: `GPU_TRIP_C` is 91.0 (watchdog.rs:25), not 95. Hard backstop untouched, so Should-fix not Blocking.
  fix-shape hint: add both keys to `sanitized()` with an `is_finite` check and a clamp into (ambient, `GPU_TRIP_C` − band) / a sane NVMe range, warning on fallback; one config test per key.

- [Should-fix] src/fanctrl/client.rs:207 — `print all` is deserialized strictly across every strategy in fw-fanctrl's dumped configuration, so one unrelated strategy entry missing `movingAverageInterval` fails the whole poll and drops the controller out of Mode A permanently, though only the active strategy's curve is used.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓)
  evidence: `StrategyField { moving_average_interval: u32, speed_curve: Vec<_> }` non-Option inside `HashMap<String, StrategyField>`; `parse_print_all` (client.rs:258-260) parses the whole map before the active lookup. Grounded upstream: fw-fanctrl's config.schema.json `required: ["speedCurve"]` only (https://github.com/TamtamHero/fw-fanctrl/blob/main/src/fw_fanctrl/_resources/config.schema.json), `Configuration.parse` stores raw JSON with no default-filling, and `FanController.dump_details` returns `vars(self.configuration)` verbatim — so a schema-valid user strategy omitting the optional key produces exactly this shape on every `print all` → `all_observed_at` stale after 90 s → `FanctrlLost`/Mode B. Local install (fw-fanctrl 0.0.0-11.20260606) matches.
  fix-shape hint: make `StrategyField`'s fields `Option`/`#[serde(default)]` (default MA interval 20 per upstream), or deserialize `strategies` as `HashMap<String, serde_json::Value>` and parse only the active entry.

- [Should-fix] src/control/sim_tests.rs:2179-2200; src/control/sim_tests.rs:531 — No closed-loop acceptance simulation ever attaches a GPU actuator or GPU watts LUT (all 31 `build_controller` calls pass `with_gpu: false`, every `run_ticks` passes `gpu_lut: None`), so the GPU PI, `verify_lock` and the actuator write path are never exercised in a plant-closed run; the "dGPU on" coverage axis is satisfied by a single tick scripting a sensor temperature.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓)
  evidence: `grep -c 'false)'` = 31, `'true)'` = 0; `run_ticks` doc (sim_tests.rs:515-517) admits `gpu_lut` is None everywhere; controller.rs:2000-2050 gates the NVML write + `verify_lock` inside `match self.guard.gpu { None => {} … }`; `grep Unverifiable src/control/sim_tests.rs` = 0 hits though fwloop.17 acceptance (design doc line 958) explicitly requires "verify_lock `Unverifiable`" and "each spec-enumerated configuration (… dGPU on/off …) is exercised end to end". `build_controller` already supports `with_gpu: true` with `FakeGpu`. Reproduce-seat correction: the GPU HOT ratchet (`gpu_share_override`) does run in the 88 °C closed-loop test since it needs no actuator — the gap is the PI/write/verify_lock chain. Kept Should-fix: NVML clock locks are a real hardware side-effect and this is the branch's own acceptance bar.
  fix-shape hint: run the dGPU-powered/hot and dGPU-unpowered scenarios with `with_gpu: true` + a scripted `gpu_lut`, asserting `GpuSet` effects and the `verify_lock` verdict.

- [Nit] src/fanctrl/curve.rs:63 — `Curve::from_points` validates only that duty is non-decreasing, not temperature, so a curve whose points are out of temperature order is accepted, `tread`/`t_star` steer Mode A to a setpoint where the forward `duty_at` yields a lower duty, and `CURVE INVALID` is never raised.
  verdict: confirmed (reproduce ✓ / refute ✗ (rejected: upstream has the same assumption) / ground ✓)
  evidence: curve.rs:59-67 compares only `.1`; reproduce seat executed the worked example (`[(0,15),(70,20),(50,25),(90,30)]` → `t_star(27)=70`, `continuous_duty_at(70)=20`); client.rs:234-278 takes points verbatim with no sort. Design doc §2.1 gives "T* could land on a falling segment" as the rationale for the check, which this shape defeats. Refute seat: upstream FanController.py (https://raw.githubusercontent.com/TamtamHero/fw-fanctrl/main/src/fw_fanctrl/FanController.py) also assumes ascending temperature with no validation, so such a config is malformed for the whole ecosystem.
  demoted: profile states single-operator with own config and hardware backstops; the trigger is an operator-authored curve that fw-fanctrl itself mishandles identically, so Nit here.
  fix-shape hint: add `w[1].0 < w[0].0` to the same loop as a second `CurveError` variant.

- [Nit] src/control/controller.rs:1497 — The §2.6 reconciliation skip-rule slope is computed per-sample (`/ (len-1)`) rather than per-second, and the window is only pushed when `s.ec` is `Some`, so an EC dropout decouples the divisor from wall time. **Correction to the claim:** the refute seat showed the direction is the opposite of what was claimed — a gap makes the window span *more* than `len-1` seconds, so the slope is *over*stated and the rule *over*-skips (delays scoring), not under-skips.
  verdict: confirmed (reproduce ✗ (rejected: direction backwards) / refute ✗-survived / ground ✓)
  evidence: controller.rs:1497 `(last - first) / (auto.ec_slope_window.len() - 1) as f64`; push gated at 1441-1448; `EcReading::read` (ec.rs:73-100) legitimately returns `None` on a per-tick read failure; design doc §2.6 line 586 states the threshold in °C/s; the same code is duplicated at 2262-2278. Refute-seat arithmetic: entries at t=0,1,3,4,5 give len=5, divisor 4, true span 5 → computed > true.
  demoted: real mechanism, but the corrected consequence (delayed reconciliation scoring on a rare dropout) is observability-grade under a single-operator profile.
  fix-shape hint: store `(t_mono, max_c)` pairs and divide by `t_last − t_first`; share the helper with the calib copy.

- [Nit] README.md:71,168,176,204; src/config.rs:55; src/control/controller.rs:248 — User-facing and in-code docs still carry the pre-a78 thresholds (`gpu_hot_c` 90 / exit 85, GPU emergency 87) while the shipped constants are 88 / 86 / 91 — the README describes exactly the soft-above-hard inversion a78 fixed.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓)
  evidence: guards.rs:38 `GPU_HOT_C_DEFAULT = 88.0`, :48 `GPU_HYSTERESIS_C = 2.0`, watchdog.rs:25 `GPU_TRIP_C = 91.0`, config.rs:425 test asserts 88; commit cb4a2ab changed constants and touched the design doc but not README.md (its `--stat` lists no README), and the stale README lines were themselves added by this branch.
  demoted: profile states a single operator with no external readers; runtime constants and their test are correct, so a stale README/doc-comment is a Nit here (safety-adjacent, fix alongside the §2.8 spec text).
  fix-shape hint: search-and-replace 90/85/87/"− 5" in README.md, config.rs:55, controller.rs:248 and design doc §2.8 header to 88/86/91/"− 2".

- [Nit] src/sensors/poller.rs:70,76; src/fanctrl/client.rs:448 — Every fw-fanctrl socket failure (ENOENT, refused, timeout, parse failure) is discarded without a log line at any level; the classified `FanctrlError` text is built and thrown away.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓)
  evidence: poller.rs:70/76 `let _ = src.poll(...)` with the comment "a poll failure is swallowed here"; `grep tracing src/fanctrl/*.rs` empty; every sibling sensor on master logs its failures (gpu.rs:48-71, cpu.rs:35-106, hwmon.rs:28-133).
  demoted: profile states observability polish is low-value for this single-operator tool; degradation is already surfaced as `FANCTRL LOST`.
  fix-shape hint: `tracing::warn!` on transition into failure (rate-limited or edge-triggered) carrying `FanctrlError`'s Display.

- [Nit] src/control/controller.rs:2835; src/control/controller.rs:2794-2803 — The `freeze` column of the `decision` telemetry line is hardcoded `None` in the only production emitter; the live value in `Effect::AutoAllocated.freeze` is discarded by the shell's `..` destructure, contradicting design §2.4/§3.5 and the field's own doc.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓)
  evidence: controller.rs:2815-2836 ends `freeze: None,`; 1967-1974 pushes `freeze: freeze.map(freeze_str)`; 2794-2803 destructures `{ demand_cpu, demand_gpu, cpu_w, gpu_w, .. }`; integration_tests.rs:586-637 hand-builds a literal and cannot fail; design doc line 331 "Freeze is reported in the decision telemetry".
  demoted: profile states observability is low-value here; the freeze itself behaves correctly, only its offline visibility is lost.
  fix-shape hint: destructure `freeze` from the effect (or add it to `ControlStatus`) and pass it into the Decision record; assert it in the emitter-level test.

- [Nit] src/control/controller.rs:2817 — The `decision` line's `mode` column carries the controller `Mode`, never the `LoopMode`, so which loop was regulating is not a column.
  verdict: confirmed (reproduce ✓ / refute ✗ (rejected: reconstructible per line) / ground ✓)
  evidence: `mode: status.mode.as_str()`; `ControlStatus.loop_mode` (338) unused there; `Effect::AutoAllocated.mode: LoopMode` dropped via `..`; design doc lines 721/808/815 use "mode" for LoopMode. Refute seat: per-line reconstruction exists — `t_star` present ⇒ TempLoop, absent + no `sensor_lost` ⇒ RpmLoop, absent + `sensor_lost` ⇒ Released (mode.rs:470-483) — so the "replay from file start" claim is overstated.
  demoted: observability, single-operator profile, and a per-line proxy already exists.
  fix-shape hint: add a `loop_mode` column to `Record::Decision` from `status.loop_mode`.

- [Nit] src/control/mode.rs:261,487-512; src/control/controller.rs:1659 — `Decision.reasons` has no production consumer, so `fanctrl_inactive`, `argmax_uncontrollable`, `unreconciled` and `ec_invalid` reach neither log, telemetry nor UI despite design doc line 276 stating `unreconciled` "appears in the telemetry".
  verdict: confirmed (reproduce ✗ (rejected: fwloop.15 enumerates the surface without `reasons`) / refute ✗-survived / ground ✓)
  evidence: `grep -rn '\.reasons' src/` outside mode.rs tests is empty; `mirror_decision` syncs six flags only; telemetry `Decision` has no reasons field. The reproduce seat's point that fwloop.8/15 never list a flag or column for these is fair, but design doc line 276 is an explicit promise.
  demoted: observability, single-operator profile.
  fix-shape hint: either join `reasons` into a `reason` column on the decision line, or add the missing StatusFlags — and reconcile the spec either way.

- [Nit] src/selftest.rs:57-176 — `selftest` gained no check of the fw-fanctrl socket or the `cros_ec` EC read, so a machine where Mode A can never engage still reports an all-OK preflight.
  verdict: confirmed (reproduce ✓ / refute ✗ (rejected: TUI flags cover it) / ground ✓)
  evidence: `git diff master...HEAD -- src/selftest.rs` only rewrites `cpu_limit_step`; no step opens `config.fanctrl_socket` or probes `cros_ec`; mode.rs:457-460 gates `hard_ok` on both. Refute seat: `FANCTRL LOST`/`EC MISMATCH` are shown live in the TUI and logged, so the operator is not diagnosis-blind once running.
  demoted: observability/preflight polish, single-operator profile, safe Mode B fallback.
  fix-shape hint: two informational selftest steps: one `print speed` round trip, one `find_chip_dir(cros_ec)`.

- [Nit] src/control/controller.rs:2527; src/state.rs — The state-file migration is one-way: the first Auto exit rewrites `state.json` without `model`, so rolling back to the `master` binary gates Auto on `model.is_none()` until a full recalibration; no `.bak`, reverse direction undocumented.
  verdict: confirmed (reproduce ✗ (rejected: profile scopes state.json as non-valuable) / refute ✗-survived / ground ✓)
  evidence: `save_persisted_state` builds the v2 struct; `write_atomic` (config.rs:232-245) replaces in place; master gates Auto on `model.is_none() || lut.is_none()`; design doc fwloop.11 only tests v1→v2. Floor check: not "real data" — a regenerable calibration cache (15-40 min per master's own README), and the project's own prior roast profile calls state.json non-valuable.
  demoted: profile states rollback is a git revert on the operator's own machine and there is no external data; a forced recalibration on downgrade is a Nit.
  fix-shape hint: write `state.json.bak` once on first v2 save, and note the one-way migration in state.rs/README.

- [Nit] README.md:248-255 — README's closing Status paragraph says the closed loop is not yet wired and `a` still drives the prior allocator; tasks 19/20 landed that wiring later on this branch and INDEX.md marks the design `implemented`.
  verdict: confirmed (reproduce ✓ / refute ✗-survived / ground ✓)
  evidence: README written at 39a1950 (09:34); a66971e/dbfeecd (12:48) and f81d3f1/91b32e5 (13:42) are descendants; controller.rs:1375 `on_auto_sample` runs guards, arbiter and `budget.step`.
  demoted: docs-only, single-operator profile.
  fix-shape hint: replace the Status paragraph with the shipped state.

- [Nit] README.md:15-24 — The Requirements section never states fw-fanctrl must be installed and running with its JSON command socket, though the rewritten intro makes it central.
  verdict: confirmed (reproduce ✓ / refute ✗ (rejected: covered under Configuration/Status) / ground ✓)
  evidence: Requirements hunk byte-identical to master; `fanctrl_socket` added in config.rs; 15+ README mentions describe behaviour when it is absent but none say it is a prerequisite. Refute seat: README:73/155/175 already document the socket and the `FANCTRL LOST` fallback.
  demoted: docs-only, single-operator profile.
  fix-shape hint: one bullet: "fw-fanctrl (≥ the version whose `print all` JSON is captured in tests/fixtures/fanctrl/) running, socket at `fanctrl_socket`; without it the loop stays in Mode B."

## Not verified (beyond panel cap)
- none

## Beyond remainder cap (count only)
- none

## Rejected (with reason)
- src/sensors/poller.rs:72-78 — due-times advance by `+= PERIOD` and fire a catch-up burst after a gap. Rejected 1/3: both refute-side seats disproved the named triggers — suspend (Rust Linux `Instant` is CLOCK_MONOTONIC per rust-lang/rust#88714 closed unmerged; design doc:270-271 assumes the same) and timing-out polls (the due-time advances after every attempt, success or failure; the blocked tick consumes the time). Only an unquantified scheduler stall remains. The `+=` pattern is a minor robustness nit; see Escalations for the clock-semantics contradiction this surfaced.
- src/fanctrl/client.rs:331-343 — one leaked connect thread per timeout, no cap/backoff. Rejected 1/3: the leak is documented in the diff's own doc comment (client.rs:320-330) as an accepted std-only tradeoff for the pathological full-backlog case; common failures (ENOENT/ECONNREFUSED) never spawn; a poller-thread panic is already reaped (main.rs:299-301); README:12-13 "No daemon … start it for a gaming session" undercuts the lifetime framing.
- src/control/controller.rs:1933-1936 — released actuator re-commanded every tick with no backoff. Rejected 3/3: design doc §2.9 lines 680-682 mandates it verbatim ("the write plus read-back continues every reassert period … otherwise no later `Verified` could ever be produced"); fwloop.12 acceptance restates it; the code comment cites it.
- src/control/budget.rs:363 — hard-freeze→DemandLimited transition skips `resync_error`. Rejected 1/3: the cited ActuatorMismatch case is resynced by a bespoke path — `in_episode()` only clears on `Verified`, which is scored `Recovered`, and `apply_verdict_outcome` (controller.rs:2104-2110) calls `resync_error` that same tick, one tick before the freeze can become DemandLimited. Calibrating uses a separate scratch `Budget`; Released re-entry starts a fresh `AutoState`. The refute seat could not fully rule out a Released→DemandLimited edge if the demand latch was already true — noted, not confirmed.
- src/control/guards.rs:102-107; src/control/watchdog.rs:83-92 — guard fails open on `None`, sensor-lost watches only CPU. Rejected 1/3: `None` deactivating the guard is spec'd verbatim (design doc:622-623) with a dedicated test; the CPU-only sensor-lost rule is byte-identical to master (commit 7c124c5) — pre-existing; NVMe guard is reporting-only by design.
- src/fanctrl/client.rs:200-230 — unversioned socket JSON, no tolerant number-or-string parsing, no version probe. Rejected 3/3: the string/number asymmetry is real and deliberate upstream (FanController.py `str(self.speed)`), but any parse failure degrades to the designed, tested `FANCTRL LOST`/RpmLoop path; docs/research/05:203-204 names this exact risk as the reason Mode B is the robust base. Documentation nit at most.
- src/control/sim_tests.rs:479-492,1131-1141 — band residency graded against `snapped_rpm`, not `fan_target_rpm`. Rejected 3/3: design doc:17 defines the acceptance bar as "within ±150 of the snapped target"; the −8 % refinement scenario is specified word-for-word (lines 866-868, 958); `DutyRpmTable::refine` has its own unit coverage.
- src/control/watchdog.rs:25 — `GPU_TRIP_C` not pinned by any test. Rejected 3/3: sim_tests.rs:2297-2360 scripts a literal 88.0 °C for 300 ticks and its post-episode assertions fail if the trip were 87 (the test's own comment records that history); controller.rs:6355-6414 likewise holds literal 88/87 below the trip.

## Unverified nits (spot-checked)
Spot-check survived (refute seat could not kill it):
- [Nit] src/control/allocator.rs:291 — `.max(floor)` re-applied after quantise but never `.min(max)`; a non-grid ceiling (e.g. the GPU HOT ratchet's `last_gpu_cap_w − DOWN_RATE_W`) can be exceeded by ≤ 0.25 W.
- [Nit] docs/superpowers/reviews/fwloop-execution/…ledger.md:43; task-2-report.md:41-42 — absolute path of the maintainer's plaintext `sudo.txt` and the `sudo -S … < sudo.txt` procedure committed (value not committed; pattern already on master in relative form).
- [FYI] .claude/settings.json; .codex/hooks.json; .codex/config.toml — repo-committed hooks run `bd` from PATH unpinned on session start / every prompt (disclosed in CLAUDE.md; commit 9a5307c flags it for review).
- [Nit] CLAUDE.md:70-77; AGENTS.md; .agents/skills/beads/SKILL.md — agent-tooling bundled into the functional branch; CLAUDE.md template placeholders unfilled (npm example vs real `cargo` commands).
- [Nit] src/control/controller.rs:754-772,1411-1444 vs 2233-2278 — EC-replica pipeline duplicated between `AutoState` and six `calib_*` fields (the per-sample slope defect is duplicated too).
- [Nit] src/control/spike_antiwindup.rs:56-79 vs src/control/budget.rs:70-91 — spike harness uses private copies of the demand-limited predicate/margins/dwell; nothing asserts the two constant sets agree, and budget.rs's own tests use a 12 W gap that would pass any margin in (0, 12).
- [Nit] src/control/controller.rs:281-308; src/ui/view.rs:186-273 — flag severity encoded three times with no agreement test; code records one prior drift.
- [Nit] src/control/controller.rs:1375-2070 — Auto loop lives on `Controller` not `AutoState`: 39 `self.auto.as_*().expect(...)` re-borrows (1 on master).
- [Nit] src/control/sim_tests.rs:2111-2238 — `Coverage<T>` checklist asserts over vectors the test itself pushed; the `dgpu_powered` axis has no behavioural assertion.
- [Nit] src/control/sim_tests.rs:1978-1994 — dGPU-unpowered test's central assertions hold by construction (`with_gpu: false`), not via the `gpu_w_valid` early return its comment credits; no test covers `gpu_w_valid=false` with a live actuator.
- [Nit] plant.rs:48; spike_antiwindup.rs:103; fopdt.rs:260; table.rs:322 — `Xorshift32` copied four times though `test_support` is the shared helper module.
- [Nit] src/control/controller.rs:1993 — `run_gpu_pi` clones `ClockWattsLut` per sample; a split borrow compiles (seat verified with `cargo check`), master borrowed it.
- [Nit] src/fanctrl/client.rs:258-274,240-251 — `parse_print_all` parses the 8.8 KB body twice (once inside `resolve_curve`) while the fanctrl mutex is held.
- [Nit] src/telemetry.rs:54,129; src/types.rs — v2 `sample` line carries the NVMe reading twice (`nvme_temp_c` and `nvme_c`) plus internal `ec_valid`/`fanctrl_view_changed`, frozen at the schema bump.
- [Nit] src/control/mode.rs:366-424 — `TargetUnreachable` raised from four conditions distinguishable only in the discarded `reasons`.
- [Nit] src/sensors/sampler.rs:56-67,155; src/sensors/ec.rs:86,93 — missing `cros_ec` chip logged nowhere (NVML absence warns); per-read EC failures at `debug` below the default `info` filter.
- [Nit] src/config.rs:137-156; src/main.rs — misspelled `fanctrl_socket` key silently ignored and the effective path never logged; indistinguishable from fw-fanctrl not running.
- [Nit] src/fanctrl/client.rs:772-777 — read-timeout test relies on a 2 s server sleep outrunning a 200 ms client timeout (EOF race → `Other` instead of `Timeout`); only real-clock sleep race in the suite.
- [Nit] src/control/sim_tests.rs:1191-1240 — demand-starved test allows `u` to within 1 W of the ceiling and grades band residency only from the first settled sample, excluding the overshoot peak.
- [Nit] Cargo.toml:16 — `float_roundtrip` justification still cites the deleted thermal model params.
- [Nit] docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md:620-646 — §2.8 header and dGPU bullet still say 5 °C hysteresis / 90/85, contradicted by the a78 revision paragraph ten lines below (same root as the README threshold finding above).
- [Nit] TODO.md:1-3 — untouched, still lists trim/RLS/thermal-model as shipped "CODE-COMPLETE" work; fwloop.14's Files list names TODO.md for the sweep.
- [Nit] README.md:244-247 — Development section omits `docs/superpowers/specs/` where the shipped design lives.

Spot-check refuted (left for the record so re-roasts don't re-litigate):
- poller cadence test never exercises a gapped clock — spec's fwloop.9 acceptance is exactly the uniform 60 s run.
- shutdown joins poller threads without timeout — spec'd ("both threads join on shutdown", fwloop.9) and the blocking-read consequence is documented in poller.rs:127-137; hardware restore already ran.
- telemetry unbounded JSONL growth — pre-existing on master; diff only widens lines.
- `ThermalPlant::tick` round-trips through the filesystem — `EcLabel` has no public constructor by design (plant.rs:466-476 documents the tradeoff); 30-test plant suite runs in 0.03 s.
- baseline/robustness test names are configuration-only — criterion is documented on `run_baseline` and in the assert messages.
- `Curve` rebuilt up to 3x per sample — O(n) over ~16 points at 1 Hz; cache is scoped to the arbiter by fwloop.10.
- `EcReading::read` re-reads 32 label files per tick — spec'd verbatim (design doc:258-259); ~40 syscalls/s, conceded immaterial.
- fw-fanctrl `status` field never inspected — upstream error bodies are `{"status":"error","reason":…}` with none of the required fields, so they already fail as `Other`.
- `Record::Sample` fields publicly constructible — single-binary crate, one correct call site.
- `#[allow(dead_code)]` pub items / lossy `resolve_curve` — design doc integration-sweep section names each as by-design; pattern pre-exists on master.
- `PersistedState` has no schema version field — v1 had none either; downgrade never in scope.
- `AutoAllocated.error` not telemetered — §3.5 enumerates the added columns and `error` is not among them.
- `assert_ec_ma_tracks_emulator` compares same type — it is the fwloop.17 acceptance as written; boxcar semantics pinned by literal-value test ec.rs:380-393; docs/research/05 anchors upstream behaviour.
- `every_config_key_is_read_somewhere` cannot fail on a stale key — it is the compile-time exhaustiveness guard the sweep describes; the gpu_hot_c regression test lives in controller.rs:6243.
- `FakeFanctrl` duplicates `poll` semantics — fake-based seams are the spec'd test architecture (design doc:901-902); the drift-prone rule is pinned on both sides.
- `ryzenadj --info` label dependency unversioned — failure mode is bounded to informational `READBACK BLIND` by design; the RAPL stickiness watchdog independently checks the cap; `DEMAND_MARGIN_W_CPU` is a hardcoded literal, not re-parsed.
- `pid` crate retained — gpu_pid.rs unchanged from master; crate is load-bearing on every unsaturated tick.
- docs/plans/ designs lack a superseded banner — fwloop.14 acceptance explicitly permits docs/plans/ history hits.

## Escalations (need human)
- Instant/suspend clock semantics — the repo contradicts itself: design doc:270-271 says "a monotonic clock does not advance across suspend" while Cargo.toml:5 and sampler.rs:103-104 say ">= 1.87: Linux Instant is CLOCK_BOOTTIME-backed (advances during suspend)". Two seats rejected the poller catch-up finding on the 2022-closed rust-lang/rust#88714; the ground seat flagged the repo's own opposite claim. `is_resume_gap` and the resume-reassert path depend on which is true for the pinned toolchain — a human should verify against the actual rustc in use and correct whichever side is wrong (and, if BOOTTIME is real, re-open the poller catch-up-burst finding).
---

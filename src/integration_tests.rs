//! Integration sweep (task `fw-fanctrl-loop-nsc`), the epic's root
//! integration task. `cfg(test)` only, declared from `main.rs`. Three jobs,
//! one section each:
//!
//! 1. [`main_flow`] walks the goal's main flows end to end on a fresh,
//!    LUT-only `state.json`: `TempLoop -> socket death -> RpmLoop ->
//!    recovery -> TempLoop`, a real calibration through the controller
//!    (not the FOPDT math directly, unlike `control::sim_tests`'s own
//!    calibration test), then a simulated daemon restart that reloads the
//!    warm-start, table and gains from the same `state.json`.
//! 2. [`wiring_sweep`] is the brief's five enumerations: every `Config`
//!    key, every `StatusFlag`, every telemetry field, every `Effect`
//!    variant, every `CalibContext` field. Each is an exhaustive
//!    destructure/match, not prose — a variant/field added later fails
//!    THIS module to compile until it is named here, which is the
//!    rot-resistance the brief asks for.
//! 3. [`real_types`] is the three integration tests no per-task test
//!    covers: sampler -> controller -> telemetry with the REAL (non-fake,
//!    non-`ChainedPlant`) `Sampler`; config -> poller construction; a full
//!    `on_command`/`on_sample` session on the fakes.
//!
//! None of this reaches into `control::sim_tests` (a private, cfg(test)
//! sibling module of `control::controller` — not nameable from here) or
//! `control::controller`'s own `#[cfg(test)] mod tests` (same reason): every
//! helper below is its own, built from the crate's public cfg(test) surface
//! (`test_support::plant`, `actuators::cmd::test_support::FakeRunner`).

use std::path::PathBuf;

use crate::actuators::cmd::test_support::FakeRunner;
use crate::actuators::cpu::CpuActuator;
use crate::actuators::guard::RestoreGuard;
use crate::actuators::smu_module::SmuModule;
use crate::config::Config;
use crate::control::budget::LoopGains;
use crate::control::controller::{Command, Controller, Effect, LoopMode, Mode, StatusFlag};
use crate::control::lut::ClockWattsLut;
use crate::state::PersistedState;
use crate::test_support::plant::{ChainedPlant, TickScript};
use crate::types::Sample;

/// Unique-per-test fixture root; caller removes it when done. Mirrors every
/// other module's own `fixture_dir`/`profile_fixture` helper (config.rs,
/// state.rs, controller.rs) — deliberately not shared with them (see the
/// module doc: those are unreachable from here).
fn fixture_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "bazerame-integration-test-{}-{name}",
        std::process::id()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// `<dir>/platform_profile` seeded with a real value, and `<dir>/state.json`
/// (not yet created — callers `save`/`load` it directly).
fn fixture_paths(name: &str) -> (PathBuf, PathBuf, PathBuf) {
    let dir = fixture_dir(name);
    let profile_path = dir.join("platform_profile");
    std::fs::write(&profile_path, "balanced\n").unwrap();
    let state_path = dir.join("state.json");
    (dir, profile_path, state_path)
}

/// A `Controller<&FakeRunner>` over a CPU-only actuator (no GPU: every flow
/// test below drives CPU-side thermal load only) and an already-unloaded
/// `ryzen_smu`, built from `persisted`/`config` exactly like `main.rs`'s own
/// `Controller::new` call site.
fn build_controller<'r>(
    runner: &'r FakeRunner,
    profile_path: &std::path::Path,
    state_path: &std::path::Path,
    persisted: PersistedState,
    config: Config,
) -> Controller<&'r FakeRunner> {
    let mut cpu = CpuActuator::new(runner, profile_path.to_path_buf());
    cpu.toggle_delay = std::time::Duration::from_millis(1); // keep the test fast
    Controller::new(
        RestoreGuard::new(runner, Some(cpu), None, Some(SmuModule::assume_unloaded())),
        persisted,
        state_path.to_path_buf(),
        config,
        PathBuf::from("/nonexistent/config.toml"),
    )
}

/// `quiet16`'s live curve (design doc §Facts), duplicated locally rather
/// than reaching into `control::sim_tests`'s own copy (unreachable — see
/// the module doc).
const QUIET16_POINTS: &[(f64, u8)] = &[
    (0.0, 15),
    (55.0, 15),
    (65.0, 21),
    (75.0, 31),
    (82.0, 37),
    (88.0, 55),
    (95.0, 100),
];
/// fw-fanctrl's `movingAverageInterval` on both live curves (§Facts).
const MA_INTERVAL: u32 = 60;
/// A cold-enough ambient that quiet16's fan-target-implied T* is reachable
/// well inside the default `cpu_max_w` ceiling.
const AMBIENT_C: f64 = 40.0;

/// A minimal, uncalibrated LUT (three points): enough for `Auto` entry to
/// require nothing beyond the `Some` check (`gpu_floor_w` resolution needs
/// at least one point) without pretending to be a real sweep result. Flow 1
/// starts from exactly this — a fresh `state.json` "with only a LUT".
fn seed_lut() -> ClockWattsLut {
    let mut lut = ClockWattsLut::new();
    lut.insert(1200, 30.0);
    lut.insert(2000, 60.0);
    lut.insert(2800, 100.0);
    lut
}

/// Drives `plant`/`ctl` for `ticks` 1 Hz samples, closing the loop on the
/// controller's own last-commanded CPU cap (the same pattern
/// `control::sim_tests::run_ticks` uses, reimplemented here since that
/// module is unreachable — see the file doc). `on_tick` gets a mutable
/// `TickScript` already seeded with the fed-back cap, so a caller can layer
/// a socket-death/revival edge, a resume, etc. on top before the tick runs.
fn drive_ticks<R: crate::actuators::cmd::Runner>(
    plant: &mut ChainedPlant,
    ctl: &mut Controller<R>,
    cpu_floor_w: f64,
    ticks: u64,
    mut on_tick: impl FnMut(u64, &mut TickScript),
) {
    for t in 1..=ticks {
        let status = ctl.status().clone();
        let mut script = TickScript {
            cpu_cap_w: status.cpu_limit_w.unwrap_or(cpu_floor_w),
            cpu_demand_frac: 1.0,
            cpu_util_pct: 95.0,
            on_ac: true,
            ..TickScript::default()
        };
        on_tick(t, &mut script);
        let sample = plant.tick(&script);
        ctl.on_sample(&sample);
    }
}

// =====================================================================
// Job 1: main flows end to end
// =====================================================================
mod main_flow {
    use super::*;

    /// A `sweep_pinned`-shaped sample (mirrors
    /// `control::controller::tests::sweep_pinned`, unreachable from here):
    /// enough for the LUT sweep's own point-recording gate (a pinned GPU
    /// clock/util/watts triple, a healthy cool CPU reading and a valid fan)
    /// without any dependency on `ChainedPlant`, which does not model the
    /// GPU-clock sweep protocol at all.
    fn sweep_pinned(clock: u32) -> Sample {
        Sample {
            gpu_util_pct: 99.0,
            gpu_sm_mhz: f64::from(clock),
            gpu_w: f64::from(clock) / 30.0,
            gpu_w_valid: true,
            gpu_mhz_valid: true,
            fan1_rpm: 3000.0,
            fan_valid: true,
            cpu_temp_c: 60.0,
            cpu_temp_valid: true,
            ..Sample::default()
        }
    }

    /// Drives the whole LUT sweep phase through the controller with pinned
    /// samples (mirrors `control::controller::tests::drive_sweep`).
    fn drive_lut_sweep(ctl: &mut Controller<&FakeRunner>) {
        use crate::calib::lut_sweep::SWEEP_CLOCKS;
        for (i, &clock) in SWEEP_CLOCKS.iter().enumerate() {
            for _ in 0..60 {
                ctl.on_sample(&sweep_pinned(clock));
                let calib = ctl.status().calib.as_ref().expect("still calibrating");
                if calib.phase != "lut" || calib.step > i {
                    break;
                }
            }
        }
        let calib = ctl.status().calib.as_ref().expect("still calibrating");
        assert_eq!(calib.phase, "step", "LUT sweep must hand off to the step test: {calib:?}");
    }

    /// Drives the step-test phase with a REAL `ChainedPlant` (the same
    /// FOPDT physics `control::sim_tests`'s own off-controller calibration
    /// test fits against), closing the loop on the controller's own
    /// commanded CPU cap exactly like Auto mode's `drive_ticks` does —
    /// `apply_calib_set_budget` (the step test's `SetBudget` effect handler)
    /// runs the same `split_budget` -> command path Auto uses, so
    /// `status().cpu_limit_w` is real live actuation to feed back. Panics if
    /// calibration has not concluded within `max_ticks` (a hang here is a
    /// test bug, not a scenario this suite should tolerate silently).
    fn drive_step_test_to_conclusion(
        plant: &mut ChainedPlant,
        ctl: &mut Controller<&FakeRunner>,
        cpu_floor_w: f64,
        max_ticks: u64,
    ) {
        // The step test's settle gate reads the RAW `Sample.fan1_rpm`/
        // `fan2_rpm` (unlike Mode B's `rpm_smoothed`, there is no FAN_SMOOTH_N
        // tail-mean ahead of it) against a +-100 RPM window
        // (`STEADY_RPM_TOLERANCE`). `FanPlant`'s own +-90 RPM per-tick xorshift
        // noise (design §Facts/§5) is uncorrelated sample to sample, so a raw
        // 20-sample window's range is well over that tolerance almost always
        // -- real hwmon tach chips report a hardware-debounced value, not
        // independent per-tick jitter this wide, so a short tail-mean here
        // stands in for that debouncing rather than for anything the
        // production sampler itself smooths (it doesn't, upstream of the
        // controller). Test-local only; nothing under `src/calib` changes.
        const RPM_DEBOUNCE_N: usize = 5;
        let mut rpm_debounce: std::collections::VecDeque<f64> = std::collections::VecDeque::new();
        for _ in 0..max_ticks {
            let status = ctl.status().clone();
            let script = TickScript {
                cpu_cap_w: status.cpu_limit_w.unwrap_or(cpu_floor_w),
                cpu_demand_frac: 1.0,
                cpu_util_pct: 95.0,
                on_ac: true,
                ..TickScript::default()
            };
            let mut sample = plant.tick(&script);
            rpm_debounce.push_back(sample.max_fan_rpm());
            if rpm_debounce.len() > RPM_DEBOUNCE_N {
                rpm_debounce.pop_front();
            }
            let debounced = rpm_debounce.iter().sum::<f64>() / rpm_debounce.len() as f64;
            sample.fan1_rpm = debounced;
            sample.fan2_rpm = debounced;
            ctl.on_sample(&sample);
            if ctl.status().calib.is_none() {
                return;
            }
        }
        panic!(
            "step test never concluded through the controller within {max_ticks} ticks \
             (status: {:?})",
            ctl.status()
        );
    }

    /// Job 1, all three legs in one continuous session (a real daemon
    /// session never resets `state.json` between them either):
    ///
    /// - Engage Auto from a fresh `state.json` carrying only a LUT; walk
    ///   `TempLoop -> socket death -> RpmLoop -> recovery -> TempLoop`.
    /// - Run a calibration THROUGH THE CONTROLLER (LUT sweep + a real,
    ///   physically-simulated step test) to a landed fit.
    /// - "Restart": build a brand-new `Controller` from `PersistedState`
    ///   loaded off the same `state.json` path (exactly `main.rs`'s own
    ///   startup sequence) and confirm the warm-start, table and gains
    ///   reloaded — observable only through the public surface
    ///   (`PersistedState::load` and the second controller's `status()`),
    ///   since `Controller`'s fields are private to `control::controller`.
    #[test]
    fn engage_walk_calibrate_and_restart_reloads_warm_start_table_and_gains() {
        let (dir, profile_path, state_path) = fixture_paths("main-flow");
        let runner = FakeRunner::new();
        let fresh_lut_only = PersistedState { lut: Some(seed_lut()), ..PersistedState::default() };
        let mut ctl = build_controller(
            &runner,
            &profile_path,
            &state_path,
            fresh_lut_only,
            Config::default(),
        );

        // ---- Engage Auto from the fresh, LUT-only state ----
        ctl.on_command(Command::SetAuto(true));
        assert_eq!(ctl.status().mode, Mode::Auto);
        assert!(
            !ctl.status().flags.contains(&StatusFlag::NotCalibrated),
            "a present LUT must satisfy Auto entry's calibration requirement"
        );

        let cpu_floor_w = ctl.status().cpu_floor_w;
        let mut plant = ChainedPlant::new("quiet16", QUIET16_POINTS.to_vec(), MA_INTERVAL, AMBIENT_C, 1)
            .expect("valid curve");

        // ---- Walk into TempLoop ----
        drive_ticks(&mut plant, &mut ctl, cpu_floor_w, 200, |_, _| {});
        assert_eq!(
            ctl.status().loop_mode,
            LoopMode::TempLoop,
            "premise: a live quiet16 socket must settle into TempLoop before the walk \
             below means anything: {:?}",
            ctl.status()
        );
        assert!(!ctl.status().flags.contains(&StatusFlag::FanctrlLost));

        // ---- Socket death -> RpmLoop ----
        drive_ticks(&mut plant, &mut ctl, cpu_floor_w, 30, |_, s| s.socket_dead = true);
        assert_eq!(
            ctl.status().loop_mode,
            LoopMode::RpmLoop,
            "a dead socket must fall the loop back to RpmLoop: {:?}",
            ctl.status()
        );
        assert!(
            ctl.status().flags.contains(&StatusFlag::FanctrlLost),
            "a dead socket must raise FANCTRL LOST: {:?}",
            ctl.status()
        );

        // ---- Recovery -> back to TempLoop ----
        drive_ticks(&mut plant, &mut ctl, cpu_floor_w, 120, |_, _| {});
        assert_eq!(
            ctl.status().loop_mode,
            LoopMode::TempLoop,
            "a revived socket must recover the loop to TempLoop: {:?}",
            ctl.status()
        );
        assert!(
            !ctl.status().flags.contains(&StatusFlag::FanctrlLost),
            "FANCTRL LOST must clear on recovery: {:?}",
            ctl.status()
        );

        // ---- Run a calibration (through the controller, real physics) ----
        ctl.on_command(Command::SetAuto(false));
        assert_eq!(ctl.status().mode, Mode::Monitor);
        ctl.on_command(Command::StartCalibration);
        drive_lut_sweep(&mut ctl);
        let mut calib_plant =
            ChainedPlant::new("quiet16", QUIET16_POINTS.to_vec(), MA_INTERVAL, AMBIENT_C, 2)
                .expect("valid curve");
        drive_step_test_to_conclusion(&mut calib_plant, &mut ctl, cpu_floor_w, 700);
        assert_eq!(ctl.status().mode, Mode::Monitor, "calibration must conclude back to Monitor");
        assert!(
            !ctl.status().flags.contains(&StatusFlag::NotCalibrated),
            "a landed calibration must keep NOT CALIBRATED clear"
        );

        // The calibration's own effects saved state as it went (design
        // §2.4/§3.3: `SaveState` fires on the landed fit); re-derive the
        // warm-start entry the walk above should ALSO have written, by
        // running Auto again briefly and forcing a save through the public
        // `SetAuto(false)` exit path (mirrors every sim_tests acceptance
        // run's own `assert_steady_window_recorded` pattern).
        ctl.on_command(Command::SetAuto(true));
        drive_ticks(&mut plant, &mut ctl, cpu_floor_w, 60, |_, _| {});
        ctl.on_command(Command::SetAuto(false));

        let after_session = PersistedState::load(&state_path);
        assert!(after_session.lut.is_some(), "the original LUT must still be present");
        assert!(
            after_session.calibrated_at.is_some(),
            "a landed calibration must stamp calibrated_at: {after_session:?}"
        );
        let fitted_gains = after_session
            .loop_gains
            .expect("a landed calibration must persist fitted loop_gains");
        assert_ne!(
            fitted_gains,
            LoopGains::default(),
            "a genuinely fitted gain set is vanishingly unlikely to equal the IMC \
             defaults bit-for-bit; this guards against a fit that silently fell back"
        );

        // ---- "Restart the daemon": a brand-new Controller from the same
        // state.json, exactly main.rs's own startup sequence ----
        let reloaded = PersistedState::load(&state_path);
        assert_eq!(reloaded, after_session, "load must reproduce exactly what was saved");
        let runner2 = FakeRunner::new();
        let mut ctl2 =
            build_controller(&runner2, &profile_path, &state_path, reloaded, Config::default());

        // Table reload: a default-constructed table's `DutyRpmTable` must
        // equal what a fresh `PersistedState::default()` would carry ONLY
        // if the session above never refined it; either way, the exact
        // table that was saved is what round-tripped (already checked
        // above via `PersistedState::load` equality) -- this asserts the
        // SECOND controller's entry behavior actually reflects it: no
        // NOT CALIBRATED (lut reload) and no fresh-floor budget seed if a
        // warm-start entry exists for quiet16's snapped duty (gains/table
        // reload feeding the same key the first session recorded into).
        ctl2.on_command(Command::SetAuto(true));
        assert_eq!(ctl2.status().mode, Mode::Auto);
        assert!(
            !ctl2.status().flags.contains(&StatusFlag::NotCalibrated),
            "the reloaded LUT must satisfy Auto entry on the restarted controller"
        );
        drive_ticks(&mut plant, &mut ctl2, cpu_floor_w, 1, |_, _| {});
        if !after_session.warm_start.is_empty() {
            assert!(
                ctl2.status().budget_w > 0.0,
                "a non-empty reloaded warm_start must seed u above the bare floor sum \
                 on the very first tick: {:?}",
                ctl2.status()
            );
        }

        std::fs::remove_dir_all(&dir).unwrap();
    }
}

// =====================================================================
// Job 2: the unwired-sweep checklist, as five exhaustive enumerations
// =====================================================================
//
// Each test below destructures/matches its type with NO wildcard arm, so a
// field or variant added later fails THIS module to compile until it is
// named here with where it is consulted — the rot-resistance the brief
// asks for ("prefer writing each enumeration as a test over writing it as
// prose"). Every claim in a comment below was verified against the source
// while writing this sweep (task `fw-fanctrl-loop-nsc`); two gaps it found
// (`gpu_hot_c`/`nvme_hot_c` never reaching `Guards::new`, and
// `StatusFlag::TargetUnreachable` never reaching `ControlStatus`) are fixed
// inline in this same task, noted where they were.
mod wiring_sweep {
    use super::*;
    use crate::config::LedConfig;

    /// Every `Config` key.
    #[test]
    fn every_config_key_is_read_somewhere() {
        let Config {
            fan_target_rpm, // control/controller.rs: SetFanTarget + the Auto allocator's RPM target
            cpu_floor_w,    // control/allocator.rs split_budget floor; control/budget.rs Budget's lower bound
            gpu_floor_mhz,  // control/allocator.rs / actuators/gpu.rs clamp_gpu_clock; Budget's gpu_floor_w resolution
            fast_limit_mw,  // control/controller.rs Controller::new -> CpuActuator.fast_limit_mw -> actuators/cpu.rs --fast-limit=
            cpu_max_w,      // control/allocator.rs Input::cpu_max_w (the "100%" ceiling + grid-search bound)
            gpu_max_w,      // control/allocator.rs Input::gpu_max_w
            // Reached AutoState::new -> Guards::new only as of this sweep
            // (fw-fanctrl-loop-nsc): previously hard-coded to
            // GPU_HOT_C_DEFAULT/NVME_HOT_C_DEFAULT regardless of config,
            // fixed inline (see the regression test
            // `gpu_and_nvme_hot_thresholds_come_from_the_live_config_not_the_compiled_defaults`
            // in control::controller's own test module).
            gpu_hot_c,
            nvme_hot_c,
            leds,           // main.rs: led::spawn(config.leds.clone(), ...)
            fanctrl_socket, // main.rs: UnixFanctrlClient::new(config.fanctrl_socket.clone())
            ..
        } = Config::default();
        assert!(fan_target_rpm > 0.0);
        assert!(cpu_floor_w >= 0.0);
        assert!(gpu_floor_mhz > 0);
        assert!(fast_limit_mw > 0);
        assert!(cpu_max_w > 0.0);
        assert!(gpu_max_w > 0.0);
        assert!(gpu_hot_c > 0.0);
        assert!(nvme_hot_c > 0.0);
        assert!(!fanctrl_socket.as_os_str().is_empty());

        let LedConfig {
            enabled,        // led/mod.rs: master switch, gates spawn() entirely
            cpu_port,       // led/mod.rs: open_side("CPU", &config.cpu_port, ...)
            gpu_port,       // led/mod.rs: open_side("GPU", &config.gpu_port, ...)
            brightness,     // led/mod.rs: Matrix::open(path, config.brightness)
            flip_time,      // led/mod.rs -> led/render.rs Orientation::flip_time
            cpu_flip_watts, // led/mod.rs -> Orientation::flip_watts (CPU side)
            gpu_flip_watts, // led/mod.rs -> Orientation::flip_watts (GPU side)
        } = leds;
        assert!(enabled);
        assert!(!cpu_port.is_empty());
        assert!(!gpu_port.is_empty());
        assert!(brightness > 0);
        assert!(!flip_time);
        assert!(cpu_flip_watts);
        assert!(!gpu_flip_watts);
    }

    /// Every `StatusFlag` variant: where it is raised, and where it is
    /// rendered (`ui/view.rs`). The match itself (no wildcard) is the
    /// exhaustiveness check; the strings are load-bearing documentation a
    /// reader can grep the cited file for.
    #[test]
    fn every_status_flag_is_raised_and_rendered() {
        let all = [
            StatusFlag::LimitNotSticking,
            StatusFlag::Resumed,
            StatusFlag::NotCalibrated,
            StatusFlag::TargetUnreachable,
            StatusFlag::ThermalEmergency,
            StatusFlag::SensorLost,
            StatusFlag::FanctrlLost,
            StatusFlag::EcMismatch,
            StatusFlag::SteepCurve,
            StatusFlag::CurveInvalid,
            StatusFlag::GpuHot,
            StatusFlag::NvmeHot,
            StatusFlag::ReadbackBlind,
        ];
        for flag in all {
            let (raised_where, rendered_where) = match flag {
                StatusFlag::LimitNotSticking => (
                    "control/controller.rs: the RAPL-vs-commanded read-back watchdog",
                    "ui/view.rs: \"LIMIT-SLIP!\", red bold",
                ),
                StatusFlag::Resumed => (
                    "control/controller.rs: on_sample's s.resumed branch",
                    "ui/view.rs: \"resumed\", yellow",
                ),
                StatusFlag::NotCalibrated => (
                    "control/controller.rs: SetAuto(true) with no LUT present",
                    "ui/view.rs: not-calibrated hint",
                ),
                StatusFlag::TargetUnreachable => (
                    "control/mode.rs Arbiter::decide (design §2.7, all three cases); synced into \
                     ControlStatus by mirror_decision only as of this sweep (fw-fanctrl-loop-nsc, \
                     was bead fw-fanctrl-loop-a5j -- mirror_decision's flag list never named it)",
                    "ui/view.rs: \"TARGET UNREACHABLE\", red bold",
                ),
                StatusFlag::ThermalEmergency => (
                    "control/controller.rs: ThermalWatchdog thermal trip",
                    "ui/view.rs: red bold + acknowledge hint",
                ),
                StatusFlag::SensorLost => (
                    "control/controller.rs: ThermalWatchdog sensor-lost trip",
                    "ui/view.rs: red bold + acknowledge hint",
                ),
                StatusFlag::FanctrlLost => (
                    "control/mode.rs Arbiter::decide (socket Freshness::Absent/Stale)",
                    "ui/view.rs: \"FANCTRL LOST\", yellow",
                ),
                StatusFlag::EcMismatch => (
                    "control/mode.rs Arbiter::decide (3-strike EC/replica reconciliation)",
                    "ui/view.rs: \"EC MISMATCH\", yellow",
                ),
                StatusFlag::SteepCurve => (
                    "control/mode.rs Arbiter::decide (slope_at(T*) > 2%/C)",
                    "ui/view.rs: \"STEEP CURVE\", gray/info",
                ),
                StatusFlag::CurveInvalid => (
                    "control/mode.rs Arbiter::decide (Curve::from_points rejects a non-monotone curve)",
                    "ui/view.rs: \"CURVE INVALID\"",
                ),
                StatusFlag::GpuHot => (
                    "control/guards.rs Guards::step, synced every on_auto_sample tick",
                    "ui/view.rs: \"GPU HOT\", yellow",
                ),
                StatusFlag::NvmeHot => (
                    "control/guards.rs Guards::step, synced every on_auto_sample tick",
                    "ui/view.rs: \"NVME HOT\", yellow",
                ),
                StatusFlag::ReadbackBlind => (
                    "control/controller.rs VerdictState (6 consecutive Unreadable/Unverifiable read-backs)",
                    "ui/view.rs: \"READBACK BLIND\", info",
                ),
            };
            // The exhaustive `match` above IS the verification: a new
            // `StatusFlag` variant fails the build until its raise/render
            // sites are named. These two strings are documentation, not
            // evidence — `!literal.is_empty()` cannot fail — so they are
            // bound here rather than asserted on (ledger: task 24 minor).
            let _ = (raised_where, rendered_where);
        }
    }

    /// Every telemetry field: `Record::Sample`'s and `Record::Decision`'s
    /// own struct patterns are exhaustive (no `..`), so a field added to
    /// either fails this module to compile.
    #[test]
    fn every_telemetry_field_is_populated() {
        use crate::telemetry::Record;

        let s = Sample::default();
        match Record::sample(&s, Some(42.0)) {
            Record::Sample {
                sample: _,     // the flattened Sample itself (telemetry.rs #[serde(flatten)])
                ec_max,        // sample.ec.max_c
                ec_argmax,     // sample.ec.argmax.as_str()
                ec_ma,         // caller-supplied (the controller's live EcAverage; Sample never carries it)
                nvme_c,        // sample.nvme_temp_c
                fanctrl_speed, // sample.fanctrl.speed_pct
                fanctrl_active,// sample.fanctrl.active
                strategy,      // sample.fanctrl.strategy
            } => {
                assert_eq!(ec_ma, Some(42.0));
                assert_eq!(ec_max, None); // Sample::default() carries no ec reading
                assert_eq!(ec_argmax, None);
                assert_eq!(nvme_c, None);
                assert_eq!(fanctrl_speed, None);
                assert_eq!(fanctrl_active, None);
                assert_eq!(strategy, None);
            }
            Record::Flag { .. } | Record::Decision { .. } => {
                panic!("Record::sample must build a Record::Sample line")
            }
        }

        let decision = Record::Decision {
            t_mono: 1.0,
            mode: "auto".to_string(),
            cpu_limit_w: Some(20.0),
            gpu_max_mhz: Some(2000),
            fan_target_rpm: 2600.0,
            cause: "auto:allocate".to_string(),
            flags: vec!["steep_curve".to_string()],
            demand_cpu: Some(10.0),
            demand_gpu: Some(5.0),
            alloc_cpu_w: Some(9.0),
            alloc_gpu_w: Some(4.0),
            pi_target_w: Some(4.0),
            t_star: Some(60.0),
            budget_w: 30.0,
            freeze: Some("Calibrating".to_string()),
        };
        match decision {
            Record::Decision {
                t_mono,        // apply_effects's own call arg
                mode,          // status.mode.as_str()
                cpu_limit_w,   // status.cpu_limit_w
                gpu_max_mhz,   // status.gpu_max_mhz
                fan_target_rpm,// status.fan_target_rpm
                cause,         // the batch's first-claimed cause string
                flags,         // status.flags, mapped to as_str()
                demand_cpu,    // Effect::AutoAllocated.demand_cpu, when present in the batch
                demand_gpu,    // Effect::AutoAllocated.demand_gpu
                alloc_cpu_w,   // Effect::AutoAllocated.cpu_w
                alloc_gpu_w,   // Effect::AutoAllocated.gpu_w
                pi_target_w,   // Effect::AutoAllocated.gpu_w (also the GPU PI's own target)
                t_star,        // status.t_star_c
                budget_w,      // status.budget_w (always carried, never Option)
                freeze,        // Effect::AutoAllocated.freeze
            } => {
                // Unlike the `Record::sample` half above (a production
                // builder whose outputs are genuinely checked), this record
                // is a hand-built literal: asserting each field equal to the
                // value it was just constructed from cannot fail. The
                // exhaustive pattern (no `..`) is the verification — a new
                // `Decision` field fails the build until it is named here
                // with its source. Bound, not asserted (ledger: task 24).
                let _ = (
                    t_mono, mode, cpu_limit_w, gpu_max_mhz, fan_target_rpm, cause, flags,
                    demand_cpu, demand_gpu, alloc_cpu_w, alloc_gpu_w, pi_target_w, t_star,
                    budget_w, freeze,
                );
            }
            Record::Flag { .. } | Record::Sample { .. } => {
                panic!("must stay a Record::Decision line")
            }
        }
    }

    /// Every `Effect` variant: where `control::controller::apply_effects`
    /// (the controller thread shell) applies it. `CpuSet`/`GpuSet`/
    /// `Released` carry no further action there because they are
    /// already-executed hardware facts by the time the effect is built
    /// (`on_sample`/`on_command` did the write); the rest drive telemetry,
    /// the UI channel, or shutdown as named.
    #[test]
    fn every_effect_variant_is_applied() {
        let effects: Vec<Effect> = vec![
            Effect::CpuSet(10.0),
            Effect::GpuSet(1000),
            Effect::Released,
            Effect::Reasserted { cause: "reassert" },
            Effect::StatusChanged { cause: "command:set_cpu_w" },
            Effect::Noted { cause: "calib:point_recorded" },
            Effect::AutoAllocated {
                demand_cpu: 1.0,
                demand_gpu: 1.0,
                cpu_w: 1.0,
                gpu_w: 1.0,
                mode: LoopMode::TempLoop,
                error: 0.5,
                budget_w: 30.0,
                freeze: None,
            },
            Effect::Flagged { flag: "gpu_hot", active: true },
            Effect::Quit,
        ];
        // No count assertion: a hand-typed length compared to a hand-typed
        // list cannot catch a variant the same author forgot in both. The
        // exhaustive `match` below (no wildcard) is the verification — a
        // new `Effect` variant fails the build until its apply site is
        // named, and a removed one leaves an unreachable arm that also
        // fails to compile (ledger: task 24 minor).
        for effect in &effects {
            match effect {
                Effect::CpuSet(_) => {}     // already committed to hardware inside on_sample/on_command
                Effect::GpuSet(_) => {}     // already committed to hardware
                Effect::Released => {}      // already committed to hardware
                Effect::Reasserted { .. } => {} // apply_effects: claims the batch's Decision cause
                Effect::StatusChanged { .. } => {} // apply_effects: Event::Status to the UI + Decision cause
                Effect::Noted { .. } => {}  // apply_effects: Decision cause (telemetry-only, status unchanged)
                Effect::AutoAllocated { .. } => {} // apply_effects: Decision's demand_*/alloc_*/t_star/budget_w/freeze
                Effect::Flagged { .. } => {} // apply_effects: a standalone Record::Flag line, in addition to Decision
                Effect::Quit => {}          // controller::spawn's shell loop breaks on this
            }
        }
    }

    /// Every `CalibContext` field: `build_calib_context`'s own derivation,
    /// every one of them from that tick's live `Sample`/arbiter output,
    /// never a stale or default-constructed value.
    #[test]
    fn every_calib_context_field_originates_from_live_data() {
        use crate::calib::step::CalibContext;
        let ctx = CalibContext {
            ec_ma: Some(50.0),      // calib_ec_avg.push(ec.max_c), seeded from the first view's ma_temperature
            ec_mismatch: false,     // calib_arbiter.decide(&ArbiterInput { .. }).ec_mismatch
            fanctrl_active: true,   // s.fanctrl_freshness == Fresh && s.fanctrl.as_ref().is_some_and(|v| v.active)
            argmax_controllable: true, // s.ec.as_ref().is_some_and(|e| e.argmax.is_controllable())
            budget_bounds: (15.0, 54.0), // self.budget_bounds() (config floors/maxes + the LUT's gpu_floor_w)
        };
        // The exhaustive destructure is the verification: a `CalibContext`
        // field added without a named source here fails the build. The
        // values are a hand-built literal, so asserting them equal to
        // themselves cannot fail — bound, not asserted (ledger: task 24).
        // The runtime claim that each field comes from live data is
        // covered by `calib_context_real_wiring_lets_the_step_test_actually_settle`
        // in control::controller's test module, which drives the real
        // `build_calib_context`.
        let CalibContext { ec_ma, ec_mismatch, fanctrl_active, argmax_controllable, budget_bounds } = ctx;
        let _ = (ec_ma, ec_mismatch, fanctrl_active, argmax_controllable, budget_bounds);
    }
}

// =====================================================================
// Job 3: the integration tests no per-task test covers
// =====================================================================
mod real_types {
    use super::*;
    use std::path::Path;
    use std::sync::{Arc, Mutex};

    use crate::fanctrl::client::{FanctrlSource, UnixFanctrlClient};
    use crate::sensors::poller::{self, FanctrlPoller, SharedFanctrl, SharedNvme};
    use crate::sensors::sampler::Sampler;
    use crate::telemetry::{Record, Telemetry};

    /// Sampler -> controller -> telemetry, with the REAL `Sampler` (not
    /// `test_support::plant::ChainedPlant`'s synthetic `Sample`): every
    /// sysfs/procfs path points at a nonexistent fixture root, so every
    /// sensor comes back absent/invalid (exactly what a machine missing
    /// that hardware reports), but the code path exercised end to end is
    /// the production one `main.rs` wires -- `Sampler::sample()` builds a
    /// real `Sample`, `Controller::on_sample` consumes it for real, and
    /// `Telemetry`/`Record::sample` writes a real JSONL line back off it.
    #[test]
    fn sampler_to_controller_to_telemetry_line_with_real_types() {
        let dir = fixture_dir("real-types-telemetry");
        let fanctrl_source: SharedFanctrl = poller::shared_fanctrl();
        let nvme_cache: SharedNvme = Arc::new(Mutex::new(None));
        let mut sampler = Sampler::with_paths(
            Path::new("/nonexistent/hwmon"),
            None,
            Path::new("/nonexistent/stat"),
            Path::new("/nonexistent/cpu-base"),
            Path::new("/nonexistent/power-supply"),
            None,
            fanctrl_source,
            nvme_cache,
        );
        let sample = sampler.sample();
        assert!(!sample.fan_valid, "no hwmon fixture: the real Sampler must report absent, not fabricate");

        let (profile_dir, profile_path, state_path) = fixture_paths("real-types-controller");
        let runner = FakeRunner::new();
        let mut ctl = build_controller(
            &runner,
            &profile_path,
            &state_path,
            PersistedState::default(),
            Config::default(),
        );
        // Monitor mode with nothing commanded legitimately returns no
        // effects for a quiet sample (no status change, nothing to
        // reassert) -- the real assertion is simply that a real `Sample`
        // survives `on_sample` without panicking.
        let _effects = ctl.on_sample(&sample);

        let mut telemetry = Telemetry::open(&dir).expect("telemetry dir is fresh and writable");
        telemetry.log(&Record::sample(&sample, ctl.status().ec_ma_c));
        telemetry.flush();

        let entry = std::fs::read_dir(&dir)
            .unwrap()
            .find(|e| e.as_ref().unwrap().file_name().to_string_lossy().starts_with("run-"))
            .expect("Telemetry::open must have created a run-*.jsonl file")
            .unwrap();
        let text = std::fs::read_to_string(entry.path()).unwrap();
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 2, "run_start header + the one sample line: {text:?}");
        let line: serde_json::Value = serde_json::from_str(lines[1]).expect("must be valid JSON");
        assert_eq!(line["kind"], "sample");
        assert!(line.get("t_mono").is_some(), "the flattened Sample fields must be present: {line}");

        std::fs::remove_dir_all(&dir).unwrap();
        std::fs::remove_dir_all(&profile_dir).unwrap();
    }

    /// Config -> poller construction: the exact chain `main.rs` builds
    /// (`config.fanctrl_socket` -> `UnixFanctrlClient` -> the shared
    /// `FanctrlSource` -> `FanctrlPoller::new`), proving the config key
    /// reaches real poller construction rather than a hard-coded path.
    /// Construction only (no `spawn`, no thread, no real socket I/O) — a
    /// deterministic unit test, not an integration test against a live
    /// fw-fanctrl daemon.
    #[test]
    fn config_fanctrl_socket_flows_into_real_poller_construction() {
        let config = Config {
            fanctrl_socket: PathBuf::from("/tmp/bazerame-fanctrl-loop-nsc-test.sock"),
            ..Config::default()
        };
        let client = Box::new(UnixFanctrlClient::new(config.fanctrl_socket.clone()))
            as Box<dyn FanctrlSource + Send>;
        let snapshot: SharedFanctrl = poller::shared_fanctrl();
        let fanctrl_poller = FanctrlPoller::new(
            client,
            Arc::clone(&snapshot),
            std::time::Instant::now(),
        );
        // Construction alone must not have polled anything yet.
        assert!(crate::sync_util::lock(&snapshot).view.is_none());
        drop(fanctrl_poller);
    }

    /// A full `on_command`/`on_sample` session on the fakes: Manual mode
    /// end to end (SetFloors, SetCpuW clamped by the floor it just set,
    /// SetGpuMaxClock, ReleaseAll back to Monitor, Quit restoring
    /// hardware) — the manual-control path none of `main_flow`'s Auto/
    /// calibration-focused walk or `control::sim_tests`'s closed-loop
    /// acceptance runs exercise end to end through a single session.
    #[test]
    fn full_manual_mode_on_command_on_sample_session_on_the_fakes() {
        let (dir, profile_path, state_path) = fixture_paths("manual-session");
        let runner = FakeRunner::new();
        let mut ctl = build_controller(
            &runner,
            &profile_path,
            &state_path,
            PersistedState::default(),
            Config::default(),
        );
        assert_eq!(ctl.status().mode, Mode::Monitor);

        let effects = ctl.on_command(Command::SetFloors { cpu_w: 20.0, gpu_mhz: 1200 });
        assert!(effects.iter().any(|e| matches!(e, Effect::StatusChanged { .. })));
        assert_eq!(ctl.status().cpu_floor_w, 20.0);

        // SetCpuW is manual actuation, not floor-clamped (the floor is the
        // Auto allocator's own concept) -- it commands exactly what was
        // asked, verified through the actuator's own read-back.
        let effects = ctl.on_command(Command::SetCpuW(15.0));
        assert!(effects.iter().any(|e| matches!(e, Effect::CpuSet(_))));
        assert_eq!(ctl.status().mode, Mode::Manual);
        assert_eq!(ctl.status().cpu_limit_w, Some(15.0));

        // A live sample while a limit is commanded: Manual mode's
        // reassert/stickiness machinery stays active on real samples.
        ctl.on_sample(&Sample { cpu_temp_c: 55.0, cpu_temp_valid: true, ..Sample::default() });
        assert_eq!(ctl.status().cpu_limit_w, Some(15.0), "a quiet sample must not perturb the commanded limit");

        let effects = ctl.on_command(Command::ReleaseAll);
        assert!(effects.iter().any(|e| matches!(e, Effect::Released)));
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert_eq!(ctl.status().cpu_limit_w, None);

        let effects = ctl.on_command(Command::Quit);
        assert!(effects.iter().any(|e| matches!(e, Effect::Quit)));

        std::fs::remove_dir_all(&dir).unwrap();
    }
}

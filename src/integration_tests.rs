//! Integration sweep (task `fw-fanctrl-loop-eb9.17`), the epic's root
//! integration task. `cfg(test)` only, declared from `main.rs`. Three jobs,
//! one section each:
//!
//! 1. [`main_flow`] walks the goal's main flows end to end on a fresh,
//!    legacy `state.json`: Curve regulation, socket loss into Held, recovery
//!    to Curve, and a real calibration through the controller
//!    (not the FOPDT math directly, unlike `control::sim_tests`'s own
//!    calibration test), then a simulated daemon restart that reloads the
//!    warm-start, table and gains from the same `state.json`.
//! 2. [`wiring_sweep`] contains four local enumerations: every `Config` key,
//!    every `StatusFlag`, every telemetry field, and every `Effect` variant.
//!    Each is an exhaustive destructure/match, so a variant or field added later fails
//!    THIS module to compile until it is named here, which is the
//!    rot-resistance the brief asks for.
//!    `control::controller::tests::calibration_context_exhaustively_uses_live_controller_and_sample_sources`
//!    owns the exhaustive `PerDeviceCalibContext` provenance check because
//!    the builder is private to the controller.
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
use crate::control::controller::{Command, Controller, Effect, Mode, StatusFlag};
use crate::types::TelemetryTStarState;
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

/// Drives `plant`/`ctl` for `ticks` 1 Hz samples, closing the loop on the
/// controller's own last-commanded CPU cap (the same pattern
/// used by the focused controller simulations). `on_tick` gets a mutable
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
    use crate::actuators::gpu::test_support::FakeGpu;
    use crate::fanctrl::client::{FanctrlView, Freshness};
    use crate::state::warm_start_key;
    use crate::types::GainsSource;

    /// Drives shared settle and both device steps with a REAL `ChainedPlant` (the same
    /// FOPDT physics `control::sim_tests`'s own off-controller calibration
    /// test fits against), closing the loop on the controller's own
    /// commanded CPU cap and GPU lock exactly like Auto mode's `drive_ticks`
    /// does. The per-device runner's direct effects use the controller's
    /// normal actuator paths, so status is real live actuation to feed back. Panics if
    /// calibration has not concluded within `max_ticks` (a hang here is a
    /// test bug, not a scenario this suite should tolerate silently).
    fn drive_step_test_to_conclusion(
        plant: &mut ChainedPlant,
        ctl: &mut Controller<&FakeRunner>,
        cpu_floor_w: f64,
        max_ticks: u64,
    ) {
        // The step test's settle gate reads the RAW `Sample.fan1_rpm`/
        // `fan2_rpm` (unlike Held's `rpm_smoothed`, there is no FAN_SMOOTH_N
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
                gpu_lock_mhz: status.gpu_max_mhz.map(f64::from),
                gpu_load_level: Some(1.0),
                gpu_powered: Some(true),
                gpu_util_pct: 95.0,
                gpu_temp_c: Some(42.0),
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
    /// - Engage Auto from a fresh `state.json` carrying only legacy fields; walk
    ///   `Curve -> socket death -> Held -> recovery -> Curve`.
    /// - Run a calibration THROUGH THE CONTROLLER (shared settle plus real,
    ///   physically simulated CPU-watt and GPU-clock steps) to landed fits.
    /// - "Restart": load the state through the public surface and confirm
    ///   migration drops retired fields while retaining revision-4 records.
    #[test]
    fn engage_walk_calibrate_and_restart_migrates_legacy_state_safely() {
        let (dir, profile_path, state_path) = fixture_paths("main-flow");
        let runner = FakeRunner::new();
        let fresh_state = PersistedState::default();
        let mut ctl = build_controller(
            &runner,
            &profile_path,
            &state_path,
            fresh_state,
            Config::default(),
        );

        // ---- Engage Auto from the fresh, legacy-only state ----
        ctl.on_command(Command::SetAuto(true));
        assert_eq!(ctl.status().mode, Mode::Auto);
        assert!(
            ctl.status().flags.contains(&StatusFlag::NotCalibrated),
            "Auto remains available with safe defaults when fitted device gains are absent"
        );

        let cpu_floor_w = ctl.status().cpu_floor_w;
        let mut plant = ChainedPlant::new("quiet16", QUIET16_POINTS.to_vec(), MA_INTERVAL, AMBIENT_C, 1)
            .expect("valid curve");

        // ---- Walk into Curve ----
        drive_ticks(&mut plant, &mut ctl, cpu_floor_w, 200, |_, _| {});
        assert_eq!(
            ctl.status().tstar_state,
            Some(TelemetryTStarState::Curve),
            "premise: a live quiet16 socket must settle into Curve before the walk \
             below means anything: {:?}",
            ctl.status()
        );
        assert!(!ctl.status().flags.contains(&StatusFlag::FanctrlLost));

        // ---- Socket death -> Held ----
        drive_ticks(&mut plant, &mut ctl, cpu_floor_w, 30, |_, s| s.socket_dead = true);
        assert_eq!(
            ctl.status().tstar_state,
            Some(TelemetryTStarState::Held),
            "a dead socket must hold the last good target: {:?}",
            ctl.status()
        );
        assert!(
            ctl.status().flags.contains(&StatusFlag::FanctrlLost),
            "a dead socket must raise FANCTRL LOST: {:?}",
            ctl.status()
        );

        // ---- Recovery -> back to Curve ----
        drive_ticks(&mut plant, &mut ctl, cpu_floor_w, 120, |_, _| {});
        assert_eq!(
            ctl.status().tstar_state,
            Some(TelemetryTStarState::Curve),
            "a revived socket must recover the loop to Curve: {:?}",
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
        let persisted_before_calibration = PersistedState::load(&state_path);
        let mut cpu = CpuActuator::new(&runner, profile_path.clone());
        cpu.toggle_delay = std::time::Duration::from_millis(1);
        let calibration_config = Config { gpu_floor_mhz: 1_200, ..Config::default() };
        ctl = Controller::new(
            RestoreGuard::new(
                &runner,
                Some(cpu),
                Some(Box::new(FakeGpu::new())),
                Some(SmuModule::assume_unloaded()),
            ),
            persisted_before_calibration,
            state_path.clone(),
            calibration_config,
            PathBuf::from("/nonexistent/config.toml"),
        );
        ctl.on_command(Command::StartCalibration);
        assert_eq!(
            ctl.status().calib.as_ref().map(|progress| progress.phase.as_str()),
            Some("settle"),
            "revision-4 calibration starts directly with the shared settle"
        );
        let mut calib_plant =
            ChainedPlant::new("quiet16", QUIET16_POINTS.to_vec(), MA_INTERVAL, AMBIENT_C, 2)
                .expect("valid curve");
        // The pair is restored and physically re-settled between the CPU
        // and GPU responses so CPU cool-down cannot masquerade as GPU-step
        // cross coupling. Leave enough deterministic plant time for that
        // recovery in addition to both 360 s response windows.
        drive_step_test_to_conclusion(&mut calib_plant, &mut ctl, cpu_floor_w, 1_600);
        assert_eq!(ctl.status().mode, Mode::Monitor, "calibration must conclude back to Monitor");
        let persisted_after_calibration = PersistedState::load(&state_path);
        assert!(
            !ctl.status().flags.contains(&StatusFlag::NotCalibrated),
            "a landed calibration must keep NOT CALIBRATED clear: {persisted_after_calibration:?}"
        );

        let target_duty = persisted_after_calibration
            .duty_rpm_table
            .duty_for_rpm(Config::default().fan_target_rpm);
        let warm_key = warm_start_key("quiet16", target_duty, true);

        // Produce the warm pair through the live controller path. Start from
        // deliberately interior manual caps, then hold both feedback groups
        // and the fan on target for the complete qualification window.
        ctl.on_command(Command::SetCpuW(31.0));
        ctl.on_command(Command::SetGpuMaxClock(2_100));
        ctl.on_command(Command::SetAuto(true));
        for i in 0..45 {
            let target_c = ctl
                .status()
                .t_star_c
                .or_else(|| persisted_after_calibration.t_star_last_good.as_ref().map(|seed| seed.value_c))
                .expect("calibration must provide a qualified target");
            let mut sample = calib_plant.tick(&TickScript {
                cpu_cap_w: 31.0,
                cpu_demand_frac: 0.45,
                cpu_util_pct: 95.0,
                gpu_lock_mhz: Some(2_100.0),
                gpu_load_level: Some(0.45),
                gpu_powered: Some(true),
                gpu_util_pct: 95.0,
                gpu_temp_c: Some(target_c),
                on_ac: true,
                ..TickScript::default()
            });
            sample.fanctrl = Some(FanctrlView {
                strategy: "quiet16".into(),
                active: true,
                speed_pct: target_duty,
                temperature: target_c,
                ma_temperature: target_c,
                ma_interval: MA_INTERVAL,
                curve: QUIET16_POINTS.to_vec(),
                observed_at: std::time::Instant::now(),
                all_observed_at: Some(std::time::Instant::now()),
            });
            sample.fanctrl_freshness = Freshness::Fresh;
            sample.fanctrl_view_changed = i == 0;
            sample.fan1_rpm = Config::default().fan_target_rpm;
            sample.fan2_rpm = Config::default().fan_target_rpm;
            sample.cpu_temp_c = target_c;
            sample.cpu_temp_valid = true;
            sample.cpu_pkg_w = 20.0;
            sample.gpu_mhz_valid = true;
            sample.gpu_sm_mhz = 1_800.0;
            if let Some(ec) = sample.ec.as_mut() {
                ec.max_c = target_c.round() as i32;
                ec.reconciliation_max_c = Some(target_c.round() as i32);
                ec.cpu_group_c = Some(target_c);
                ec.gpu_group_c = Some(target_c);
                for (label, value) in &mut ec.all {
                    *value = if label.is_controllable() { target_c } else { 40.0 };
                }
            }
            ctl.on_sample(&sample);
        }
        ctl.on_command(Command::SetAuto(false));

        let after_session = PersistedState::load(&state_path);
        assert!(
            after_session.calibrated_at.is_some(),
            "a landed calibration must stamp calibrated_at: {after_session:?}"
        );
        let saved_table = after_session.duty_rpm_table.clone();
        let gains_key = "quiet16:60";
        let cpu_gains = after_session
            .cpu_gains
            .get(gains_key)
            .expect("CPU calibration must save fitted gains under the live strategy/interval key");
        let gpu_gains = after_session
            .gpu_gains
            .get(gains_key)
            .expect("GPU calibration must save fitted gains under the live strategy/interval key");
        assert!(cpu_gains.is_valid() && gpu_gains.is_valid());
        let saved_warm_start = *after_session
            .warm_start
            .get(&warm_key)
            .expect("qualified live samples must persist a paired warm start");
        assert!(saved_warm_start.cpu_cap_w.is_finite());
        assert!(saved_warm_start.gpu_lock_mhz > 0);
        assert_ne!(saved_warm_start.cpu_cap_w, Config::default().cpu_floor_w);
        assert_ne!(saved_warm_start.cpu_cap_w, Config::default().cpu_max_w);
        assert_ne!(saved_warm_start.gpu_lock_mhz, Config::default().gpu_floor_mhz);
        assert_ne!(saved_warm_start.gpu_lock_mhz, Config::default().gpu_max_mhz);
        let saved_seed = after_session
            .t_star_last_good
            .clone()
            .expect("the live Curve session must save a qualified T* seed");
        assert_eq!(saved_seed.strategy, "quiet16");
        assert_eq!(saved_seed.fan_target_rpm, Config::default().fan_target_rpm as u32);

        // ---- "Restart the daemon": a brand-new Controller from the same
        // state.json, exactly main.rs's own startup sequence ----
        let reloaded = PersistedState::load(&state_path);
        assert_eq!(reloaded, after_session, "load must reproduce exactly what was saved");
        assert_eq!(reloaded.duty_rpm_table, saved_table);
        assert_eq!(reloaded.warm_start.get(&warm_key), Some(&saved_warm_start));
        assert_eq!(
            reloaded.t_star_last_good.as_ref().map(|seed| seed.saved_at_unix_s),
            Some(saved_seed.saved_at_unix_s),
            "loading must preserve the qualification timestamp"
        );
        let mut default_gain_state = reloaded.clone();
        default_gain_state.cpu_gains.clear();
        default_gain_state.gpu_gains.clear();
        let runner2 = FakeRunner::new();
        let mut cpu2 = CpuActuator::new(&runner2, profile_path.clone());
        cpu2.toggle_delay = std::time::Duration::from_millis(1);
        let mut ctl2 = Controller::new(
            RestoreGuard::new(
                &runner2,
                Some(cpu2),
                Some(Box::new(FakeGpu::new())),
                Some(SmuModule::assume_unloaded()),
            ),
            reloaded,
            state_path.clone(),
            Config::default(),
            PathBuf::from("/nonexistent/config.toml"),
        );

        // Table reload: a default-constructed table's `DutyRpmTable` must
        // equal what a fresh `PersistedState::default()` would carry ONLY
        // if the session above never refined it; either way, the exact
        // table that was saved is what round-tripped (already checked
        // above via `PersistedState::load` equality) -- this asserts the
        // Per-device loops consume direct watts/MHz, so the migrated v4
        // state does not need the deleted legacy table to enter Auto.
        ctl2.on_command(Command::SetAuto(true));
        assert_eq!(ctl2.status().mode, Mode::Auto);
        let restart_group_c = (saved_seed.value_c - 10.0).max(50.0);
        let mut restart_plant =
            ChainedPlant::new("quiet16", QUIET16_POINTS.to_vec(), MA_INTERVAL, AMBIENT_C, 3)
                .expect("valid restart curve");
        let mut restart_sample = restart_plant.tick(&TickScript {
            cpu_cap_w: saved_warm_start.cpu_cap_w,
            cpu_demand_frac: 1.0,
            cpu_util_pct: 95.0,
            gpu_lock_mhz: Some(f64::from(saved_warm_start.gpu_lock_mhz)),
            gpu_load_level: Some(1.0),
            gpu_powered: Some(true),
            gpu_util_pct: 95.0,
            gpu_temp_c: Some(42.0),
            on_ac: true,
            ..TickScript::default()
        });
        restart_sample.fanctrl = Some(FanctrlView {
            strategy: "quiet16".into(),
            active: true,
            speed_pct: target_duty,
            temperature: restart_group_c,
            ma_temperature: restart_group_c,
            ma_interval: MA_INTERVAL,
            curve: QUIET16_POINTS.to_vec(),
            observed_at: std::time::Instant::now(),
            all_observed_at: Some(std::time::Instant::now()),
        });
        restart_sample.fanctrl_freshness = Freshness::Fresh;
        restart_sample.fanctrl_view_changed = true;
        restart_sample.fan1_rpm = Config::default().fan_target_rpm;
        restart_sample.fan2_rpm = Config::default().fan_target_rpm;
        restart_sample.cpu_pkg_w = 10.0;
        restart_sample.gpu_mhz_valid = true;
        restart_sample.gpu_sm_mhz = 1_500.0;
        if let Some(ec) = restart_sample.ec.as_mut() {
            ec.max_c = restart_group_c.round() as i32;
            ec.reconciliation_max_c = Some(restart_group_c.round() as i32);
            ec.cpu_group_c = Some(restart_group_c);
            ec.gpu_group_c = Some(restart_group_c);
            for (label, value) in &mut ec.all {
                *value = if label.is_controllable() {
                    restart_group_c
                } else {
                    40.0
                };
            }
        }
        ctl2.on_sample(&restart_sample);
        assert_eq!(
            ctl2.status()
                .gpu
                .as_ref()
                .expect("GPU decision on keyed entry")
                .thermal
                .round() as u32,
            saved_warm_start.gpu_lock_mhz,
            "the first valid clock sample must consume the GPU half of the pair"
        );
        for i in 1..5 {
            let mut sample = restart_sample.clone();
            sample.t_mono += f64::from(i);
            sample.fanctrl_view_changed = false;
            ctl2.on_sample(&sample);
        }
        let cpu = ctl2.status().cpu.as_ref().expect("CPU decision after restart");
        let gpu = ctl2.status().gpu.as_ref().expect("GPU decision after restart");
        assert_eq!((cpu.gains_source, gpu.gains_source), (GainsSource::Fitted, GainsSource::Fitted));
        assert_eq!(cpu.thermal, saved_warm_start.cpu_cap_w);
        assert_eq!(ctl2.status().t_star_c, Some(saved_seed.value_c));
        assert_eq!(ctl2.status().tstar_state, Some(TelemetryTStarState::Held));
        assert!(!ctl2.status().flags.contains(&StatusFlag::NotCalibrated));

        // Prove the fitted label above reflects the gains actually installed
        // in DeviceLoop. A control controller gets the same pair, target and
        // samples but has no persisted gains. After one PI update its thermal
        // candidate must diverge; a missing/no-op `set_gains` would make the
        // two traces identical while still reporting `Fitted`.
        let runner3 = FakeRunner::new();
        let mut cpu3 = CpuActuator::new(&runner3, profile_path.clone());
        cpu3.toggle_delay = std::time::Duration::from_millis(1);
        let mut default_ctl = Controller::new(
            RestoreGuard::new(
                &runner3,
                Some(cpu3),
                Some(Box::new(FakeGpu::new())),
                Some(SmuModule::assume_unloaded()),
            ),
            default_gain_state,
            dir.join("default-gain-control-state.json"),
            Config::default(),
            PathBuf::from("/nonexistent/config.toml"),
        );
        default_ctl.on_command(Command::SetAuto(true));
        for i in 0..10 {
            let mut sample = restart_sample.clone();
            sample.t_mono += f64::from(i);
            sample.fanctrl_view_changed = i == 0;
            default_ctl.on_sample(&sample);
            if i >= 5 {
                ctl2.on_sample(&sample);
            }
        }
        assert_eq!(
            default_ctl
                .status()
                .cpu
                .as_ref()
                .expect("default CPU decision")
                .gains_source,
            GainsSource::Default
        );
        let fitted_thermal = ctl2.status().cpu.as_ref().expect("fitted CPU decision").thermal;
        let default_thermal = default_ctl
            .status()
            .cpu
            .as_ref()
            .expect("default CPU decision")
            .thermal;
        assert!(
            (fitted_thermal - default_thermal).abs() > 0.01,
            "persisted fitted gains must change the live PI response: fitted={fitted_thermal}, default={default_thermal}"
        );

        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn long_gap_resume_reasserts_and_holds_both_live_device_caps() {
        let (dir, profile_path, state_path) = fixture_paths("long-gap-resume");
        let runner = FakeRunner::new();
        let mut cpu = CpuActuator::new(&runner, profile_path);
        cpu.toggle_delay = std::time::Duration::from_millis(1);
        let mut ctl = Controller::new(
            RestoreGuard::new(
                &runner,
                Some(cpu),
                Some(Box::new(FakeGpu::new())),
                Some(SmuModule::assume_unloaded()),
            ),
            PersistedState::default(),
            state_path,
            Config::default(),
            PathBuf::from("/nonexistent/config.toml"),
        );
        ctl.on_command(Command::SetAuto(true));
        let mut plant =
            ChainedPlant::new("quiet16", QUIET16_POINTS.to_vec(), MA_INTERVAL, AMBIENT_C, 4)
                .expect("valid curve");
        let cpu_floor_w = ctl.status().cpu_floor_w;
        drive_ticks(&mut plant, &mut ctl, cpu_floor_w, 30, |_, script| {
            script.gpu_lock_mhz = Some(1_800.0);
            script.gpu_load_level = Some(1.0);
            script.gpu_powered = Some(true);
            script.gpu_util_pct = 95.0;
            script.gpu_temp_c = Some(42.0);
        });
        let before_cpu = ctl.status().cpu_limit_w.expect("CPU cap before resume");
        let before_gpu = ctl.status().gpu_max_mhz.expect("GPU cap before resume");
        let mut resumed = plant.tick(&TickScript {
            cpu_cap_w: before_cpu,
            cpu_demand_frac: 1.0,
            cpu_util_pct: 95.0,
            gpu_lock_mhz: Some(f64::from(before_gpu)),
            gpu_load_level: Some(1.0),
            gpu_powered: Some(true),
            gpu_util_pct: 95.0,
            gpu_temp_c: Some(42.0),
            on_ac: true,
            ..TickScript::default()
        });
        resumed.t_mono += 7_200.0;
        resumed.resumed = true;

        let effects = ctl.on_sample(&resumed);

        assert!(
            effects.iter().any(|effect| matches!(effect, Effect::Reasserted { cause: "resume" })),
            "resume must immediately reassert the applied pair: {effects:?}"
        );
        assert_eq!(ctl.status().cpu_limit_w, Some(before_cpu));
        assert_eq!(ctl.status().gpu_max_mhz, Some(before_gpu));
        let cpu = ctl.status().cpu.as_ref().expect("CPU resume decision");
        let gpu = ctl.status().gpu.as_ref().expect("GPU resume decision");
        assert_eq!(cpu.cap, before_cpu);
        assert_eq!(gpu.cap.round() as u32, before_gpu);
        assert_ne!(cpu.hold, crate::types::TelemetryHold::ActuatorMismatch);
        assert_ne!(gpu.hold, crate::types::TelemetryHold::ActuatorMismatch);

        std::fs::remove_dir_all(dir).unwrap();
    }
}

// =====================================================================
// Job 2: four exhaustive public-surface enumerations
// =====================================================================
//
// Each test below destructures or matches its type with no wildcard arm, so a
// field or variant added later fails THIS module to compile until it is
// named here with where it is consulted — the rot-resistance the brief
// asks for. The controller's private calibration-context builder is covered
// exhaustively in its own test module.
mod wiring_sweep {
    use super::*;
    use crate::config::LedConfig;

    fn code(source: &str) -> String {
        source
            .lines()
            .filter(|line| !line.trim_start().starts_with("//"))
            .collect::<Vec<_>>()
            .join(" ")
            .split_whitespace()
            .collect::<Vec<_>>()
            .join(" ")
    }

    /// Every `Config` key.
    #[test]
    fn every_config_key_is_read_somewhere() {
        let Config {
            fan_target_rpm, // controller target and shared T* source
            cpu_floor_w,    // CPU DeviceLoop and hot-guard lower bound
            gpu_floor_mhz,  // GPU DeviceLoop, actuator, and hot-guard lower bound
            fast_limit_mw,  // control/controller.rs Controller::new -> CpuActuator.fast_limit_mw -> actuators/cpu.rs --fast-limit=
            cpu_max_w,               // CPU DeviceLoop and actuator ceiling
            gpu_max_mhz,             // GPU DeviceLoop, actuator, and hot-guard ceiling
            shadow_headroom_cpu_w,   // CPU DeviceLoop shadow ceiling
            shadow_headroom_gpu_mhz, // GPU DeviceLoop shadow ceiling
            shadow_fall_rate_cpu,    // CPU DeviceLoop shadow down-slew
            shadow_fall_rate_gpu,    // GPU DeviceLoop shadow down-slew
            gpu_shadow_enabled,      // GPU DeviceLoop shadow selector
            cpu_hot_c,               // T* feasibility, CPU hot debounce and ratchet
            cpu_gains,               // CPU DeviceLoop gains source
            gpu_gains,               // GPU DeviceLoop gains source
            gpu_hot_c,               // GPU hot guard and ratchet recovery threshold
            nvme_hot_c,              // NVMe reporting guard threshold
            leds,           // main.rs: led::spawn(config.leds.clone(), ...)
            fanctrl_socket, // main.rs: UnixFanctrlClient::new(config.fanctrl_socket.clone())
        } = Config::default();
        let controller = code(include_str!("control/controller.rs"));
        let main = code(include_str!("main.rs"));
        for (field, production_use) in [
            ("fan_target_rpm", "self.config.fan_target_rpm"),
            ("cpu_floor_w", "floor: self.config.cpu_floor_w"),
            ("gpu_floor_mhz", "floor: f64::from(self.config.gpu_floor_mhz)"),
            ("fast_limit_mw", "cpu.fast_limit_mw = config.fast_limit_mw"),
            ("cpu_max_w", "cpu.set_sustained_max_mw((config.cpu_max_w * 1000.0) as u32)"),
            ("gpu_max_mhz", "f64::from(config.gpu_max_mhz)"),
            ("shadow_headroom_cpu_w", "shadow_headroom: self.config.shadow_headroom_cpu_w"),
            ("shadow_headroom_gpu_mhz", "shadow_headroom: self.config.shadow_headroom_gpu_mhz"),
            ("shadow_fall_rate_cpu", "shadow_fall_rate: self.config.shadow_fall_rate_cpu"),
            ("shadow_fall_rate_gpu", "shadow_fall_rate: self.config.shadow_fall_rate_gpu"),
            ("gpu_shadow_enabled", "shadow_enabled: self.config.gpu_shadow_enabled"),
            ("cpu_hot_c", "cpu_hot_c: self.config.cpu_hot_c"),
            ("cpu_gains", "cpu_gains_source: if config.cpu_gains.is_some()"),
            ("gpu_gains", "gpu_gains_source: if config.gpu_gains.is_some()"),
            ("gpu_hot_c", "Guards::new(config.gpu_hot_c, config.nvme_hot_c)"),
            ("nvme_hot_c", "Guards::new(config.gpu_hot_c, config.nvme_hot_c)"),
        ] {
            assert!(controller.contains(production_use), "Config::{field} lost its controller consumer: {production_use}");
        }
        assert!(main.contains("config.leds.clone()"), "Config::leds lost its runtime consumer");
        assert!(main.contains("config.fanctrl_socket.clone()"), "Config::fanctrl_socket lost its runtime consumer");
        let _ = (fan_target_rpm, cpu_floor_w, gpu_floor_mhz, fast_limit_mw, cpu_max_w,
            gpu_max_mhz, shadow_headroom_cpu_w, shadow_headroom_gpu_mhz,
            shadow_fall_rate_cpu, shadow_fall_rate_gpu, gpu_shadow_enabled, cpu_hot_c,
            cpu_gains, gpu_gains, gpu_hot_c, nvme_hot_c, fanctrl_socket);

        let LedConfig {
            enabled,        // led/mod.rs: master switch, gates spawn() entirely
            cpu_port,       // led/mod.rs: open_side("CPU", &config.cpu_port, ...)
            gpu_port,       // led/mod.rs: open_side("GPU", &config.gpu_port, ...)
            brightness,     // led/mod.rs: Matrix::open(path, config.brightness)
            flip_time,      // led/mod.rs -> led/render.rs Orientation::flip_time
            cpu_flip_watts, // led/mod.rs -> Orientation::flip_watts (CPU side)
            gpu_flip_watts, // led/mod.rs -> Orientation::flip_watts (GPU side)
        } = leds;
        let led = code(include_str!("led/mod.rs"));
        for (field, production_use) in [
            ("enabled", "if !config.enabled"),
            ("cpu_port", "open_side(\"CPU\", &config.cpu_port"),
            ("gpu_port", "open_side(\"GPU\", &config.gpu_port"),
            ("brightness", "Matrix::open(Path::new(path), config.brightness)"),
            ("flip_time", "flip_time: config.flip_time"),
            ("cpu_flip_watts", "flip_watts: config.cpu_flip_watts"),
            ("gpu_flip_watts", "flip_watts: config.gpu_flip_watts"),
        ] {
            assert!(led.contains(production_use), "LedConfig::{field} lost its runtime consumer: {production_use}");
        }
        let _ = (enabled, cpu_port, gpu_port, brightness, flip_time, cpu_flip_watts, gpu_flip_watts);
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
            let producer = match flag {
                StatusFlag::LimitNotSticking => "self.add_flag(StatusFlag::LimitNotSticking)",
                StatusFlag::Resumed => "self.add_flag(StatusFlag::Resumed)",
                StatusFlag::NotCalibrated => "self.sync_bool_flag(StatusFlag::NotCalibrated, !fitted)",
                StatusFlag::TargetUnreachable => "self.sync_bool_flag( StatusFlag::TargetUnreachable, target.flags.iter()",
                StatusFlag::ThermalEmergency => "Trip::Thermal => (StatusFlag::ThermalEmergency",
                StatusFlag::SensorLost => "Trip::SensorLost => (StatusFlag::SensorLost",
                StatusFlag::FanctrlLost => "self.sync_bool_flag( StatusFlag::FanctrlLost, !matches!",
                StatusFlag::EcMismatch => "self.sync_bool_flag(StatusFlag::EcMismatch, flags.2)",
                StatusFlag::SteepCurve => "self.sync_bool_flag( StatusFlag::SteepCurve, target .flags",
                StatusFlag::CurveInvalid => "self.sync_bool_flag( StatusFlag::CurveInvalid, view.is_some_and",
                StatusFlag::GpuHot => "self.sync_bool_flag(StatusFlag::GpuHot, flags.0)",
                StatusFlag::NvmeHot => "self.sync_bool_flag(StatusFlag::NvmeHot, flags.1)",
                StatusFlag::ReadbackBlind => "self.sync_bool_flag(StatusFlag::ReadbackBlind, blind)",
            };
            let controller = code(include_str!("control/controller.rs"));
            assert!(controller.contains(producer), "{flag:?} lost its production raise path: {producer}");

            let mut status = crate::control::ControlStatus::default();
            status.flags.push(flag);
            let wire = crate::control::controller::test_decision_telemetry_flags(&status);
            assert_eq!(wire, vec![crate::types::TelemetryFlag::legacy(flag.as_str())]);
            let json = serde_json::to_string(&wire).expect("status flag telemetry serializes");
            assert!(json.contains(flag.as_str()), "{flag:?} missing from serialized Decision flags: {json}");
            let rendered = crate::ui::view::test_status_flag_text(flag);
            assert!(!rendered.is_empty(), "{flag:?} rendered an empty TUI label");
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
                cpu_group_c,   // sample.ec.cpu_group_c
                gpu_group_c,   // sample.ec.gpu_group_c
                ec_ma,         // caller-supplied (the controller's live EcAverage; Sample never carries it)
                nvme_c,        // sample.nvme_temp_c
                fanctrl_speed, // sample.fanctrl.speed_pct
                fanctrl_active,// sample.fanctrl.active
                strategy,      // sample.fanctrl.strategy
            } => {
                assert_eq!(ec_ma, Some(42.0));
                assert_eq!(ec_max, None); // Sample::default() carries no ec reading
                assert_eq!(ec_argmax, None);
                assert_eq!(cpu_group_c, None);
                assert_eq!(gpu_group_c, None);
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
            cause: "auto:device_loops".to_string(),
            flags: vec![crate::types::TelemetryFlag::legacy("steep_curve")],
            t_star: Some(60.0),
            tstar_state: Some(crate::types::TelemetryTStarState::Held),
            cpu: Some(crate::types::TelemetryDevice {
                group_c: Some(60.0),
                err_c: Some(0.0),
                thermal: 20.0,
                shadow: 21.0,
                cap: 20.0,
                selected: crate::types::TelemetrySelected::Thermal,
                hold: crate::types::TelemetryHold::None,
                gains_source: crate::types::GainsSource::Config,
            }),
            gpu: Some(crate::types::TelemetryDevice {
                group_c: Some(60.0),
                err_c: Some(0.0),
                thermal: 2_000.0,
                shadow: 2_050.0,
                cap: 2_000.0,
                selected: crate::types::TelemetrySelected::Thermal,
                hold: crate::types::TelemetryHold::None,
                gains_source: crate::types::GainsSource::Config,
            }),
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
                t_star,        // status.t_star_c
                tstar_state,   // status.tstar_state
                cpu,           // status.cpu
                gpu,           // status.gpu
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
                    t_star, tstar_state, cpu, gpu,
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
                Effect::Flagged { .. } => {} // apply_effects: a standalone Record::Flag line, in addition to Decision
                Effect::Quit => {}          // controller::spawn's shell loop breaks on this
            }
        }
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
            fanctrl_socket: PathBuf::from("/tmp/bazerame-fanctrl-loop-eb9-17-test.sock"),
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
        // Auto controller's own concept) -- it commands exactly what was
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

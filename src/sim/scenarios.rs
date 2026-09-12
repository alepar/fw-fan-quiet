//! Closed-loop acceptance scenarios for the revision-4 per-device loops.

use std::path::PathBuf;

use crate::actuators::cmd::test_support::FakeRunner;
use crate::actuators::cpu::CpuActuator;
use crate::actuators::gpu::test_support::{FakeGpu, GpuCall};
use crate::actuators::guard::RestoreGuard;
use crate::actuators::smu_module::SmuModule;
use crate::config::Config;
use crate::control::budget::WarmStart;
use crate::control::controller::{Command, ControlStatus, Controller, Effect, Mode, StatusFlag};
use crate::control::device_loop::{
    ActuatorState, DeviceLoop, Gains, Mhz, Selected, ThermalMode, TickInput, W, default_gains,
};
use crate::control::lut::ClockWattsLut;
use crate::state::{PersistedState, TStarSeed, WarmStartEntry};
use crate::test_support::plant::{
    ChainedPlant, CpuPlantParams, GpuPlantParams, TickScript, gpu_full_load_power_w,
};
use crate::types::{
    TelemetryBound, TelemetryDevice, TelemetryDeviceName, TelemetryFlag, TelemetryHold,
    TelemetrySelected, TelemetryTStarState,
};

use super::helpers::{
    KneeSample, RelayForcingReference, RelaySample, band_residency_pct, first_sustained_in_band,
    measure_downward_knee, relay_findings,
};

const AMBIENT_C: f64 = 40.0;
const MA_INTERVAL: u32 = 60;
const FAN_TARGET_RPM: f64 = 3100.0;
// GPU thermal control may spend tens of minutes traversing the measured
// power-limit dead zone before its first downward knee crossing. That travel
// is timed separately in sim 4 and is deliberately outside sims 1-3's
// converged window.
const WARMUP_S: u64 = 60 * 60;
const CONVERGED_WINDOW_S: u64 = 30 * 60;
const QUIET16_POINTS: &[(f64, u8)] = &[
    (0.0, 15),
    (55.0, 15),
    (65.0, 21),
    (75.0, 31),
    (82.0, 37),
    (88.0, 55),
    (95.0, 100),
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LoadCase {
    Cpu,
    Gpu,
    Both,
}

impl LoadCase {
    const ALL: [Self; 3] = [Self::Cpu, Self::Gpu, Self::Both];

    fn loads(self) -> (f64, f64) {
        match self {
            Self::Cpu => (1.0, 0.18),
            Self::Gpu => (0.28, 1.0),
            Self::Both => (1.0, 1.0),
        }
    }

    fn cpu_stressed(self) -> bool {
        matches!(self, Self::Cpu | Self::Both)
    }

    fn gpu_stressed(self) -> bool {
        matches!(self, Self::Gpu | Self::Both)
    }
}

#[derive(Debug, Clone)]
struct TraceRow {
    time_s: f64,
    mode: Mode,
    rpm: f64,
    target_rpm: f64,
    t_star_c: Option<f64>,
    tstar_state: Option<TelemetryTStarState>,
    cpu: TelemetryDevice,
    gpu: TelemetryDevice,
    cpu_draw_w: f64,
    gpu_draw_w: f64,
    cpu_load: f64,
    gpu_load: f64,
    gpu_clock_mhz: f64,
    cpu_tctl_c: f64,
    gpu_temp_c: f64,
    cpu_raw_c: Option<f64>,
    gpu_raw_c: Option<f64>,
    ec_control_max_c: Option<i32>,
    ec_argmax: Option<String>,
    reconciliation_input_c: Option<i32>,
    fanctrl_temperature_c: Option<f64>,
    fanctrl_ma_c: Option<f64>,
    fanctrl_view_changed: bool,
    reconciliation_ma_c: Option<f64>,
    reconciliation_ready: bool,
    reconciliation_ever_scored: bool,
    reconciliation_mismatch: bool,
    reconciliation_scored_count: u64,
    cpu_cap_w: f64,
    gpu_cap_mhz: f64,
    cpu_guard_max: f64,
    gpu_guard_max: f64,
    cpu_actuator: ActuatorState,
    gpu_actuator: ActuatorState,
    cpu_mismatch_strikes: u8,
    gpu_mismatch_strikes: u8,
    cpu_released: bool,
    gpu_released: bool,
    flags: Vec<StatusFlag>,
    telemetry_flags: Vec<TelemetryFlag>,
    effects: Vec<Effect>,
}

#[derive(Debug)]
struct ScenarioTrace {
    rows: Vec<TraceRow>,
    samples: Vec<(f64, f64, f64, f64, f64, f64)>,
    statuses: Vec<ControlStatus>,
    gpu_calls: Vec<GpuCall>,
}

fn scratch_path(tag: &str, file: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("bzf-v4-sim-{}-{tag}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("simulation scratch directory");
    dir.join(file)
}

fn gpu_watts_lut() -> ClockWattsLut {
    let mut lut = ClockWattsLut::new();
    for (mhz, watts) in [
        (1000, 45.0),
        (1197, 49.3),
        (1402, 53.5),
        (1612, 64.2),
        (1807, 75.9),
        (1995, 90.8),
        (2143, 99.4),
        (3090, 100.0),
    ] {
        lut.insert(mhz, watts);
    }
    lut
}

type SimController<'a> = Controller<&'a FakeRunner>;

fn build_controller_with_state<'a>(
    runner: &'a FakeRunner,
    tag: &str,
    config: Config,
    persisted: PersistedState,
) -> (
    SimController<'a>,
    std::sync::Arc<std::sync::Mutex<Vec<GpuCall>>>,
) {
    let profile_path = scratch_path(tag, "platform_profile");
    std::fs::write(&profile_path, "balanced\n").expect("platform profile fixture");
    let mut cpu = CpuActuator::new(runner, profile_path);
    cpu.toggle_delay = std::time::Duration::from_millis(1);
    let gpu = FakeGpu::new();
    let gpu_calls = gpu.calls();
    let controller = Controller::new(
        RestoreGuard::new(
            runner,
            Some(cpu),
            Some(Box::new(gpu)),
            Some(SmuModule::assume_unloaded()),
        ),
        persisted,
        scratch_path(tag, "state.json"),
        config,
        PathBuf::from("/nonexistent/config.toml"),
    );
    (controller, gpu_calls)
}

fn fixed_gain_config(fan_target_rpm: f64, gpu_shadow_enabled: bool) -> Config {
    Config {
        fan_target_rpm,
        cpu_gains: Some(default_gains::<W>(MA_INTERVAL)),
        gpu_gains: Some(Gains {
            kc: 2.1,
            ti_s: 15.0,
        }),
        gpu_shadow_enabled,
        ..Config::default()
    }
}

fn run_case(
    tag: &str,
    load_case: LoadCase,
    cpu_params: CpuPlantParams,
    gpu_params: GpuPlantParams,
    fan_target_rpm: f64,
    ambient_c: f64,
    seconds: u64,
) -> ScenarioTrace {
    let config = fixed_gain_config(fan_target_rpm, true);
    let (cpu_load, gpu_load) = load_case.loads();
    let trace = run_profile(
        tag,
        config,
        cpu_params,
        gpu_params,
        ambient_c,
        seconds,
        move |_tick, status, config| {
            let mut script = TickScript {
                cpu_cap_w: status.cpu_limit_w.unwrap_or(config.cpu_floor_w),
                cpu_demand_frac: cpu_load,
                cpu_util_pct: 100.0 * cpu_load,
                gpu_powered: Some(true),
                gpu_util_pct: 100.0 * gpu_load,
                cpu_tctl_c: Some(70.0),
                gpu_temp_c: Some(70.0),
                ..TickScript::default()
            };
            if load_case == LoadCase::Cpu {
                // A lightly used GPU reports a low clock independently of
                // its high lock. Keep draw low while using the two-node
                // thermal plant, so Shadow has a strict actuator-unit margin
                // above the observed 1000 MHz input.
                let requested_lock = f64::from(status.gpu_max_mhz.unwrap_or(config.gpu_floor_mhz));
                // Preserve physical draw=load*P_full(requested lock) while
                // reporting the independently measured idle clock.
                script.gpu_cap_w = gpu_full_load_power_w(requested_lock);
                script.gpu_demand_frac = gpu_load;
                script.gpu_sm_mhz = 1000.0;
            } else {
                let lock_mhz = f64::from(status.gpu_max_mhz.unwrap_or(config.gpu_floor_mhz));
                script.gpu_lock_mhz = Some(lock_mhz);
                script.gpu_load_level = Some(gpu_load);
            }
            script
        },
    );
    for pair in trace.rows.windows(2) {
        let requested_lock_mhz = pair[0].gpu_cap_mhz;
        let expected_draw_w = gpu_load * gpu_full_load_power_w(requested_lock_mhz);
        assert!(
            (pair[1].gpu_draw_w - expected_draw_w).abs() <= 0.01,
            "[{tag}] physical GPU draw telemetry at t={:.0}: measured {:.3}W != load {gpu_load:.2} * P_full(requested {:.0}MHz) {:.3}W",
            pair[1].time_s,
            pair[1].gpu_draw_w,
            requested_lock_mhz,
            expected_draw_w,
        );
    }
    trace
}

fn run_profile(
    tag: &str,
    config: Config,
    cpu_params: CpuPlantParams,
    gpu_params: GpuPlantParams,
    ambient_c: f64,
    seconds: u64,
    mut script_for: impl FnMut(u64, &ControlStatus, &Config) -> TickScript,
) -> ScenarioTrace {
    run_profile_control(
        tag,
        config,
        cpu_params,
        gpu_params,
        ambient_c,
        seconds,
        move |tick, _plant, controller, config| script_for(tick, controller.status(), config),
    )
}

fn run_profile_control(
    tag: &str,
    config: Config,
    cpu_params: CpuPlantParams,
    gpu_params: GpuPlantParams,
    ambient_c: f64,
    seconds: u64,
    script_for: impl FnMut(u64, &mut ChainedPlant, &mut SimController<'_>, &Config) -> TickScript,
) -> ScenarioTrace {
    run_profile_control_with_state(
        tag,
        config,
        cpu_params,
        gpu_params,
        ambient_c,
        seconds,
        PersistedState {
            lut: Some(gpu_watts_lut()),
            duty_rpm_table: Default::default(),
            ..PersistedState::default()
        },
        0,
        script_for,
    )
}

#[allow(clippy::too_many_arguments)]
fn run_profile_control_with_state(
    tag: &str,
    config: Config,
    cpu_params: CpuPlantParams,
    gpu_params: GpuPlantParams,
    ambient_c: f64,
    seconds: u64,
    persisted: PersistedState,
    auto_at: u64,
    mut script_for: impl FnMut(u64, &mut ChainedPlant, &mut SimController<'_>, &Config) -> TickScript,
) -> ScenarioTrace {
    let runner = FakeRunner::new();
    let (mut controller, gpu_calls) =
        build_controller_with_state(&runner, tag, config.clone(), persisted);
    let mut plant = ChainedPlant::new(
        "quiet16",
        QUIET16_POINTS.to_vec(),
        MA_INTERVAL,
        ambient_c,
        0xEB9,
    )
    .expect("quiet16 curve");
    plant.set_cpu_plant_params(cpu_params);
    plant.set_gpu_plant_params(gpu_params);
    if auto_at == 0 {
        controller.on_command(Command::SetAuto(true));
        assert_eq!(controller.status().mode, Mode::Auto, "[{tag}] Auto entry");
    }

    let mut rows = Vec::with_capacity(seconds as usize);
    let mut samples = Vec::with_capacity(seconds as usize);
    let mut statuses = Vec::with_capacity(seconds as usize);
    for tick in 0..seconds {
        if tick + 1 == auto_at {
            controller.on_command(Command::SetAuto(true));
            assert_eq!(controller.status().mode, Mode::Auto, "[{tag}] Auto entry");
        }
        let script = script_for(tick + 1, &mut plant, &mut controller, &config);
        statuses.push(controller.status().clone());
        let sample = plant.tick(&script);
        samples.push((
            sample.t_mono,
            sample.max_fan_rpm(),
            sample.cpu_pkg_w,
            sample.gpu_sm_mhz,
            script.cpu_cap_w,
            script.gpu_lock_mhz.unwrap_or(script.gpu_sm_mhz),
        ));
        let cpu_raw_c = sample.ec.as_ref().and_then(|ec| ec.cpu_group_c);
        let gpu_raw_c = sample.ec.as_ref().and_then(|ec| ec.gpu_group_c);
        let ec_control_max_c = sample.ec.as_ref().map(|ec| ec.max_c);
        let ec_argmax = sample.ec.as_ref().map(|ec| ec.argmax.as_str().to_owned());
        let reconciliation_input_c = sample.ec.as_ref().and_then(|ec| ec.reconciliation_max_c);
        let fanctrl_temperature_c = sample.fanctrl.as_ref().map(|view| view.temperature);
        let fanctrl_ma_c = sample.fanctrl.as_ref().map(|view| view.ma_temperature);
        let fanctrl_view_changed = sample.fanctrl_view_changed;
        let effects = controller.on_sample(&sample);
        let status = controller.status().clone();
        let emitted_telemetry_flags = controller.test_emitted_telemetry_flags();
        statuses.push(status.clone());
        let Some(cpu) = status.cpu.clone() else {
            assert_ne!(
                status.mode,
                Mode::Auto,
                "[{tag}] Auto row missing CPU decision"
            );
            continue;
        };
        let gpu = status.gpu.clone().unwrap_or_else(|| {
            panic!(
                "[{tag}] GPU decision missing at tick {}, mode={:?}, flags={:?}",
                tick + 1,
                status.mode,
                status.flags
            )
        });
        let diagnostics = controller
            .test_diagnostics()
            .expect("Auto row must have diagnostics");
        rows.push(TraceRow {
            time_s: sample.t_mono,
            mode: status.mode,
            rpm: sample.max_fan_rpm(),
            target_rpm: status.fan_target_rpm,
            t_star_c: status.t_star_c,
            tstar_state: status.tstar_state,
            cpu_cap_w: status.cpu_limit_w.unwrap_or(cpu.cap),
            gpu_cap_mhz: status.gpu_max_mhz.map_or(gpu.cap, f64::from),
            cpu_guard_max: diagnostics.cpu_guard_max,
            gpu_guard_max: diagnostics.gpu_guard_max,
            cpu_actuator: diagnostics.cpu_actuator,
            gpu_actuator: diagnostics.gpu_actuator,
            cpu_mismatch_strikes: diagnostics.cpu_mismatch_strikes,
            gpu_mismatch_strikes: diagnostics.gpu_mismatch_strikes,
            cpu_released: diagnostics.cpu_released,
            gpu_released: diagnostics.gpu_released,
            cpu,
            gpu,
            cpu_draw_w: sample.cpu_pkg_w,
            gpu_draw_w: sample.gpu_w,
            cpu_load: sample.cpu_util_pct / 100.0,
            gpu_load: sample.gpu_util_pct / 100.0,
            gpu_clock_mhz: sample.gpu_sm_mhz,
            cpu_tctl_c: sample.cpu_temp_c,
            gpu_temp_c: sample.gpu_temp_c,
            cpu_raw_c,
            gpu_raw_c,
            ec_control_max_c,
            ec_argmax,
            reconciliation_input_c,
            fanctrl_temperature_c,
            fanctrl_ma_c,
            fanctrl_view_changed,
            reconciliation_ma_c: diagnostics.reconciliation_ma_c,
            reconciliation_ready: diagnostics.reconciliation_ready,
            reconciliation_ever_scored: diagnostics.reconciliation_ever_scored,
            reconciliation_mismatch: diagnostics.reconciliation_mismatch,
            reconciliation_scored_count: diagnostics.reconciliation_scored_count,
            flags: status.flags.clone(),
            telemetry_flags: emitted_telemetry_flags,
            effects,
        });
    }
    ScenarioTrace {
        rows,
        samples,
        statuses,
        gpu_calls: gpu_calls.lock().expect("GPU calls").clone(),
    }
}

fn assert_no_relay(
    tag: &str,
    rows: &[TraceRow],
    forcing_reference: Option<(&[TraceRow], f64, f64, f64, f64)>,
) {
    let samples: Vec<_> = rows
        .iter()
        .map(|row| RelaySample::new(row.time_s, row.cpu_cap_w, row.gpu_cap_mhz, row.rpm))
        .collect();
    let reference_samples = forcing_reference.map(|(reference_rows, _, _, _, _)| {
        reference_rows
            .iter()
            .map(|row| RelaySample::new(row.time_s, row.cpu_cap_w, row.gpu_cap_mhz, row.rpm))
            .collect::<Vec<_>>()
    });
    let reference = forcing_reference.zip(reference_samples.as_deref()).map(
        |(
            (_, period_s, scored_active_from_s, reference_active_from_s, steady_after_s),
            samples,
        )| RelayForcingReference {
            samples,
            period_s,
            scored_active_from_s,
            reference_active_from_s,
            steady_after_s,
        },
    );
    let grade = relay_findings(&samples, reference)
        .unwrap_or_else(|error| panic!("[{tag}] no-relay coverage bar failed: {error:?}"));
    let first = grade.window_starts_s.first().copied().unwrap();
    let last = grade.window_starts_s.last().copied().unwrap();
    eprintln!(
        "[{tag}] no-relay coverage: {} complete 30-minute windows, origins {first:.0}..{last:.0}",
        grade.window_starts_s.len()
    );
    assert!(
        grade.findings.is_empty(),
        "[{tag}] no-relay bar failed after grading {} complete windows at origins {first:.0}..{last:.0}: {:?}",
        grade.window_starts_s.len(),
        grade.findings,
    );
}

fn assert_sims_1_to_3(tag: &str, load_case: LoadCase, trace: &ScenarioTrace) {
    assert!(
        trace.rows.len() >= CONVERGED_WINDOW_S as usize,
        "[{tag}] trace shorter than the named 30-minute converged window"
    );
    let window = &trace.rows[trace.rows.len() - CONVERGED_WINDOW_S as usize..];
    let temperature_errors = |cpu: bool| -> Vec<f64> {
        window
            .iter()
            .map(|row| {
                let group = if cpu {
                    row.cpu.group_c
                } else {
                    row.gpu.group_c
                }
                .unwrap_or_else(|| {
                    panic!(
                        "[{tag}] stressed {} group missing at t={}",
                        if cpu { "CPU" } else { "GPU" },
                        row.time_s
                    )
                });
                group - row.t_star_c.expect("Curve T* must remain live")
            })
            .collect()
    };
    if load_case.cpu_stressed() {
        let errors = temperature_errors(true);
        let residency = band_residency_pct(&errors, 1.0);
        let min = errors.iter().copied().fold(f64::INFINITY, f64::min);
        let max = errors.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        assert!(
            residency >= 90.0,
            "[{tag}] CPU-at-T* bar: {residency:.2}% < 90%, error range {min:.2}..{max:.2}, final={:?}",
            window.last().unwrap().cpu
        );
    } else {
        let shadowed = window
            .iter()
            .filter(|row| {
                (row.cpu.cap - row.cpu.shadow).abs() < 0.01 && row.cpu.shadow > row.cpu_draw_w
            })
            .count();
        let residency = 100.0 * shadowed as f64 / window.len() as f64;
        assert!(
            residency >= 90.0,
            "[{tag}] unstressed CPU shadow-above-draw bar: {residency:.2}%"
        );
    }
    if load_case.gpu_stressed() {
        let errors = temperature_errors(false);
        let residency = band_residency_pct(&errors, 1.0);
        let min = errors.iter().copied().fold(f64::INFINITY, f64::min);
        let max = errors.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        assert!(
            residency >= 90.0,
            "[{tag}] GPU-at-T* bar: {residency:.2}% < 90%, error range {min:.2}..{max:.2}, final={:?}",
            window.last().unwrap().gpu
        );
    } else {
        let shadowed = window
            .iter()
            .filter(|row| {
                (row.gpu.cap - row.gpu.shadow).abs() < 0.01 && row.gpu.shadow > row.gpu_clock_mhz
            })
            .count();
        let residency = 100.0 * shadowed as f64 / window.len() as f64;
        assert!(
            residency >= 90.0,
            "[{tag}] unstressed GPU shadow-above-draw bar: {residency:.2}%; final decision={:?}, draw={:.1}W, clock={:.0}MHz",
            window.last().unwrap().gpu,
            window.last().unwrap().gpu_draw_w,
            window.last().unwrap().gpu_clock_mhz
        );
    }
    let fan_errors: Vec<_> = window.iter().map(|row| row.rpm - row.target_rpm).collect();
    let fan_residency = band_residency_pct(&fan_errors, 150.0);
    let fan_min = fan_errors.iter().copied().fold(f64::INFINITY, f64::min);
    let fan_max = fan_errors.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    assert!(
        fan_residency >= 90.0,
        "[{tag}] fan +/-150 RPM bar: {fan_residency:.2}%, error range {fan_min:.1}..{fan_max:.1}, final rpm/target {:.1}/{:.1}",
        window.last().unwrap().rpm,
        window.last().unwrap().target_rpm
    );
    assert_no_relay(tag, window, None);
    assert!(
        window
            .iter()
            .all(|row| row.tstar_state == Some(TelemetryTStarState::Curve)),
        "[{tag}] nominal converged window must remain in Curve"
    );
    assert!(
        window
            .iter()
            .all(|row| !row.flags.contains(&StatusFlag::GpuHot)),
        "[{tag}] GPU HOT must remain clear"
    );
    assert!(
        trace
            .gpu_calls
            .iter()
            .any(|call| matches!(call, GpuCall::Set(_))),
        "[{tag}] actual GPU actuator path was not exercised"
    );
    assert!(
        trace.rows.iter().any(|row| row
            .effects
            .iter()
            .any(|effect| matches!(effect, Effect::CpuSet(_))))
            && trace.rows.iter().any(|row| row
                .effects
                .iter()
                .any(|effect| matches!(effect, Effect::GpuSet(_)))),
        "[{tag}] both real write paths must be visible in trace evidence"
    );
}

fn run_nominal_sims_1_to_3() {
    for load_case in LoadCase::ALL {
        let tag = format!("nominal-{load_case:?}");
        let trace = run_case(
            &tag,
            load_case,
            CpuPlantParams::default(),
            GpuPlantParams::nominal(),
            FAN_TARGET_RPM,
            AMBIENT_C,
            WARMUP_S + CONVERGED_WINDOW_S,
        );
        assert_sims_1_to_3(&tag, load_case, &trace);
    }
}

fn run_cpu_robustness_matrix() {
    let nominal = CpuPlantParams::default();
    const CPU_MATRIX_TARGET_RPM: f64 = 2350.0;
    const CPU_MATRIX_AMBIENT_C: f64 = 49.5;
    let fixed_external_tuple = LoadCase::ALL.map(|load_case| {
        (
            load_case,
            CPU_MATRIX_TARGET_RPM,
            CPU_MATRIX_AMBIENT_C,
            load_case.loads(),
        )
    });
    let perturbations = [
        (
            "k-minus-50",
            CpuPlantParams {
                k_c_per_w: nominal.k_c_per_w * 0.5,
                ..nominal
            },
        ),
        (
            "k-plus-50",
            CpuPlantParams {
                k_c_per_w: nominal.k_c_per_w * 1.5,
                ..nominal
            },
        ),
        (
            "tau-minus-50",
            CpuPlantParams {
                tau_s: nominal.tau_s * 0.5,
                ..nominal
            },
        ),
        (
            "tau-plus-50",
            CpuPlantParams {
                tau_s: nominal.tau_s * 1.5,
                ..nominal
            },
        ),
        (
            "theta-minus-50",
            CpuPlantParams {
                theta_eff_s: nominal.theta_eff_s * 0.5,
                ..nominal
            },
        ),
        (
            "theta-plus-50",
            CpuPlantParams {
                theta_eff_s: nominal.theta_eff_s * 1.5,
                ..nominal
            },
        ),
    ];
    for (name, cpu_params) in perturbations {
        for (load_index, load_case) in LoadCase::ALL.into_iter().enumerate() {
            assert_eq!(
                (
                    load_case,
                    CPU_MATRIX_TARGET_RPM,
                    CPU_MATRIX_AMBIENT_C,
                    load_case.loads(),
                ),
                fixed_external_tuple[load_index],
                "CPU robustness external target/ambient/load tuple changed inside the K/tau/theta perturbations"
            );
            let tag = format!("cpu-matrix-{name}-{load_case:?}");
            let trace = run_case(
                &tag,
                load_case,
                cpu_params,
                GpuPlantParams::nominal(),
                CPU_MATRIX_TARGET_RPM,
                CPU_MATRIX_AMBIENT_C,
                WARMUP_S + CONVERGED_WINDOW_S,
            );
            assert_sims_1_to_3(&tag, load_case, &trace);
        }
    }
}

fn run_gpu_robustness_matrix() {
    const GPU_MATRIX_TARGET_RPM: f64 = 3380.0;
    const GPU_MATRIX_AMBIENT_C: f64 = 42.0;
    let fixed_external_tuple = LoadCase::ALL.map(|load_case| {
        (
            load_case,
            GPU_MATRIX_TARGET_RPM,
            GPU_MATRIX_AMBIENT_C,
            load_case.loads(),
        )
    });
    for tau_s in [8.0, 15.0, 25.0, 50.0] {
        for k_c_per_mhz in [0.01, 0.02, 0.03] {
            for theta_eff_s in [45.0, 90.0, 135.0] {
                let gpu_params = GpuPlantParams {
                    tau_s,
                    k_c_per_mhz,
                    theta_eff_s,
                    robustness_heat_pivot_w: Some(99.4),
                };
                for (load_index, load_case) in LoadCase::ALL.into_iter().enumerate() {
                    assert_eq!(
                        (
                            load_case,
                            GPU_MATRIX_TARGET_RPM,
                            GPU_MATRIX_AMBIENT_C,
                            load_case.loads(),
                        ),
                        fixed_external_tuple[load_index],
                        "GPU robustness external target/ambient/load tuple changed inside the K/tau/theta cross-product"
                    );
                    let tag = format!(
                        "gpu-matrix-tau{tau_s}-k{k_c_per_mhz}-theta{theta_eff_s}-{load_case:?}"
                    );
                    let trace = run_case(
                        &tag,
                        load_case,
                        CpuPlantParams::default(),
                        gpu_params,
                        GPU_MATRIX_TARGET_RPM,
                        GPU_MATRIX_AMBIENT_C,
                        WARMUP_S + CONVERGED_WINDOW_S,
                    );
                    assert_sims_1_to_3(&tag, load_case, &trace);
                }
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct StepVariant {
    name: &'static str,
    step_at_s: u64,
    total_s: u64,
    warm: bool,
    shadow_enabled: bool,
    draw_missing_until_s: Option<u64>,
}

fn assert_shadow_disable_boundary() {
    let mut loop_ = DeviceLoop::<Mhz>::new(Gains {
        kc: 2.1,
        ti_s: 15.0,
    });
    loop_.seed_candidates(3090.0, 2200.0, Some(2200.0), 10.0);
    let input = |shadow_enabled| TickInput {
        t_star: 70.0,
        group_c: Some(60.0),
        draw: Some(1900.0),
        floor: 1000.0,
        max: 3090.0,
        mode: ThermalMode::Regulate,
        actuator: ActuatorState::Verified,
        dt_s: 1.0,
        resumed: false,
        delta_tstar: 0.0,
        shadow_headroom: 300.0,
        shadow_fall_rate: 105.0,
        shadow_enabled,
    };
    let before = loop_.tick(input(true));
    loop_.note_applied(before.cap);
    let disabled = loop_.tick(input(false));
    assert_eq!(
        before.selected,
        Selected::Shadow,
        "[sim4-shadow-disable-boundary] enabled side must be Shadow-bound"
    );
    assert_eq!(
        disabled.shadow, 3090.0,
        "[sim4-shadow-disable-boundary] disabled side must remove the shadow ceiling"
    );
    assert_ne!(
        disabled.selected,
        Selected::Shadow,
        "[sim4-shadow-disable-boundary] real enabled->disabled boundary must leave Shadow selection"
    );
    assert!(
        disabled.cap >= before.cap && disabled.cap - before.cap <= 105.0 + 1.0,
        "[sim4-shadow-disable-boundary] adjacent applied continuity: enabled={:.0}, disabled={:.0}",
        before.cap,
        disabled.cap,
    );
}

fn assert_write_slew(tag: &str, rows: &[TraceRow]) {
    let mut prior_change: Option<&TraceRow> = None;
    for row in rows {
        if let Some(prior) = prior_change {
            let delta = row.gpu_cap_mhz - prior.gpu_cap_mhz;
            if delta.abs() < 0.5 {
                continue;
            }
            let elapsed = row.time_s - prior.time_s;
            let allowed_rate = if delta > 0.0 && row.gpu.selected == TelemetrySelected::Shadow {
                300.0
            } else {
                105.0
            };
            assert!(
                delta.abs() <= allowed_rate * elapsed + 1.0,
                "[{tag}] per-write slew bar at t={:.0}: selected={:?}, delta={delta:.0} MHz over {elapsed:.0}s exceeds {allowed_rate:.0} MHz/s",
                row.time_s,
                row.gpu.selected
            );
        }
        if prior_change.is_none_or(|prior| (row.gpu_cap_mhz - prior.gpu_cap_mhz).abs() >= 0.5) {
            prior_change = Some(row);
        }
    }
}

fn run_one_gpu_step(variant: StepVariant) {
    const WARM_LIGHT_DWELL_S: usize = 10;
    // Duty 30 sits on quiet16's shallow 75C segment. The requested target
    // deliberately leaves room for the plant's measured +200 RPM momentum
    // kick while preserving the specified +250 RPM crest bar.
    let target_rpm = 2700.0;
    let ambient_c = 36.0;
    let mut config = fixed_gain_config(target_rpm, variant.shadow_enabled);
    config.cpu_hot_c = 90.0;
    let trace = run_profile(
        variant.name,
        config,
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        ambient_c,
        variant.total_s,
        |tick, status, config| {
            let before_step = tick < variant.step_at_s;
            let gpu_load = if before_step {
                if variant.warm && tick + (WARM_LIGHT_DWELL_S as u64) < variant.step_at_s {
                    1.0
                } else {
                    0.15
                }
            } else {
                1.0
            };
            let draw_available = variant
                .draw_missing_until_s
                .is_none_or(|return_at| tick > return_at);
            let mut script = TickScript {
                cpu_cap_w: status.cpu_limit_w.unwrap_or(config.cpu_floor_w),
                cpu_demand_frac: 0.0,
                cpu_util_pct: 0.0,
                gpu_lock_mhz: Some(status.gpu_max_mhz.unwrap_or(config.gpu_floor_mhz).into()),
                gpu_load_level: Some(gpu_load),
                gpu_powered: Some(true),
                gpu_util_pct: if gpu_load >= 0.9 { 95.0 } else { 0.0 },
                gpu_draw_available: draw_available,
                gpu_clock_available: draw_available,
                cpu_tctl_c: Some(70.0),
                gpu_temp_c: Some(70.0),
                ..TickScript::default()
            };
            if before_step && (!variant.warm || gpu_load < 1.0) {
                // A powered light-load GPU reports its actual low clock,
                // independently of the high ceiling still available to it.
                script.gpu_lock_mhz = None;
                script.gpu_load_level = None;
                script.gpu_cap_w = 100.0;
                script.gpu_demand_frac = gpu_load;
                script.gpu_sm_mhz = 1000.0;
            }
            script
        },
    );
    let step_index = (variant.step_at_s - 1) as usize;
    let post = &trace.rows[step_index..];
    let pre = &trace.rows[..step_index];
    assert!(
        trace
            .rows
            .iter()
            .all(|row| row.gpu_draw_w > 0.0 && row.gpu_raw_c.is_some()),
        "[{}] powered-load premise through trace: physical draw and GPU group must remain present",
        variant.name,
    );
    assert!(
        post.iter()
            .take(WARM_LIGHT_DWELL_S)
            .map(|row| row.gpu_draw_w)
            .sum::<f64>()
            > pre
                .iter()
                .rev()
                .take(WARM_LIGHT_DWELL_S)
                .map(|row| row.gpu_draw_w)
                .sum::<f64>()
                * 2.0,
        "[{}] genuine light-to-heavy draw-step premise at t={} failed",
        variant.name,
        variant.step_at_s,
    );
    if variant.warm {
        assert!(
            WARM_LIGHT_DWELL_S > 0
                && (WARM_LIGHT_DWELL_S as f64) < GpuPlantParams::nominal().theta_eff_s,
            "[{}] declared powered light dwell must satisfy 0 < {}s < theta_eff={}s",
            variant.name,
            WARM_LIGHT_DWELL_S,
            GpuPlantParams::nominal().theta_eff_s,
        );
        let warm_window = &pre[pre.len() - WARM_LIGHT_DWELL_S - CONVERGED_WINDOW_S as usize
            ..pre.len() - WARM_LIGHT_DWELL_S];
        let errors: Vec<_> = warm_window
            .iter()
            .filter_map(|row| Some(row.gpu.group_c? - row.t_star_c?))
            .collect();
        assert!(
            band_residency_pct(&errors, 1.0) >= 90.0
                && warm_window
                    .iter()
                    .filter(|row| row.gpu.selected == TelemetrySelected::Thermal)
                    .count()
                    * 10
                    >= warm_window.len() * 9,
            "[{}] powered-preload warm-state premise before light dwell: group residency {:.2}%, final={:?}",
            variant.name,
            band_residency_pct(&errors, 1.0),
            warm_window.last().unwrap().gpu,
        );
        let light_boundary = &pre[pre.len() - WARM_LIGHT_DWELL_S - 1..];
        assert!(
            light_boundary
                .iter()
                .all(|row| row.mode == Mode::Auto
                    && row.tstar_state == Some(TelemetryTStarState::Curve)),
            "[{}] continuous Auto/Curve state through powered light dwell",
            variant.name,
        );
        assert!(
            light_boundary.windows(2).all(|pair| {
                (pair[1].gpu_cap_mhz - pair[0].gpu_cap_mhz).abs() <= 106.0
                    && (pair[1].gpu.thermal - pair[0].gpu.thermal).abs() <= 106.0
            }),
            "[{}] no adjacent applied-cap jump or thermal-candidate reset through powered light dwell",
            variant.name,
        );
    } else {
        assert!(
            pre.last().is_some_and(|row| row.gpu.thermal >= 3089.0),
            "[{}] cold-light premise: thermal candidate must still be at max before t={}",
            variant.name,
            variant.step_at_s,
        );
    }
    let before_cpu = trace.rows[step_index.saturating_sub(1)].cpu_cap_w;
    let cpu_step_delta = post
        .iter()
        .take(10)
        .map(|row| (row.cpu_cap_w - before_cpu).abs())
        .fold(0.0, f64::max);
    assert!(
        cpu_step_delta <= 0.51,
        "[{}] CPU-cap-isolation bar from GPU step t={}: max delta {cpu_step_delta:.2}W",
        variant.name,
        variant.step_at_s
    );

    let upward_bar =
        if variant.shadow_enabled && variant.draw_missing_until_s.is_none() && !variant.warm {
            let start = post[0].gpu_cap_mhz;
            let reached = post
                .iter()
                .find(|row| row.gpu_cap_mhz >= start + 1000.0)
                .map(|row| row.time_s);
            Some((start, reached, post[0].time_s))
        } else {
            None
        };

    let fan_crest = post
        .iter()
        .map(|row| row.rpm)
        .fold(f64::NEG_INFINITY, f64::max);
    let temp_errors: Vec<_> = post
        .iter()
        .filter_map(|row| Some((row.time_s, row.gpu.group_c? - row.t_star_c?)))
        .collect();
    let overshoot = temp_errors
        .iter()
        .map(|(_, error)| *error)
        .fold(f64::NEG_INFINITY, f64::max)
        .max(0.0);
    let overshoot_bar = if variant.warm { 2.0 } else { 4.0 };
    assert!(
        overshoot <= overshoot_bar,
        "[{}] GPU-group overshoot bar from step t={}: {overshoot:.2}C > {overshoot_bar:.1}C",
        variant.name,
        variant.step_at_s
    );
    assert!(
        post.iter()
            .all(|row| !row.flags.contains(&StatusFlag::GpuHot)),
        "[{}] no-GPU-HOT bar from step t={} failed",
        variant.name,
        variant.step_at_s
    );

    let knee_trace: Vec<_> = post
        .iter()
        .filter_map(|row| {
            Some(KneeSample::new(
                row.time_s,
                -row.gpu.err_c?,
                row.gpu.thermal,
            ))
        })
        .collect();
    let knee = measure_downward_knee(&knee_trace, 2143.0, 2.1, 15.0, 5.0)
        .unwrap_or_else(|error| panic!("[{}] measured downward-knee bar: {error:?}", variant.name));
    let hot_start_row = post
        .iter()
        .find(|row| (row.time_s - knee.hot_start_s).abs() < 0.1)
        .expect("measured hot start is in the trace");
    let plateau_row = post
        .iter()
        .find(|row| (row.time_s - knee.plateau_start_s).abs() < 0.1)
        .expect("measured plateau start is in the trace");
    let measured_distance = (plateau_row.gpu.thermal - 2143.0).max(0.0);
    assert!(
        (knee.distance_to_knee - measured_distance).abs() <= 1.0,
        "[{}] measured plateau-distance bar at t={:.0}: helper D={:.0}MHz, max(0, plateau thermal candidate {:.0}-knee 2143)={measured_distance:.0}MHz",
        variant.name,
        knee.plateau_start_s,
        knee.distance_to_knee,
        plateau_row.gpu.thermal,
    );
    let plateau_to_crossing: Vec<_> = post
        .iter()
        .filter(|row| row.time_s >= knee.plateau_start_s && row.time_s <= knee.crossing_s)
        .collect();
    assert!(
        !plateau_to_crossing.is_empty()
            && plateau_to_crossing
                .iter()
                .all(|row| row.gpu.err_c.is_some_and(|error| -error > 0.0))
            && plateau_to_crossing.windows(2).all(|pair| {
                -pair[1].gpu.err_c.expect("plateau error")
                    <= -pair[0].gpu.err_c.expect("plateau error")
            }),
        "[{}] plateau premise bar from t={:.0} through crossing t={:.0}: hot error must stay positive and non-increasing",
        variant.name,
        knee.plateau_start_s,
        knee.crossing_s,
    );
    eprintln!(
        "[{}] knee hot_start={:.0} plateau_start={:.0} crossing={:.0} D={:.0} e_min={:.3} deadline={:.0}",
        variant.name,
        knee.hot_start_s,
        knee.plateau_start_s,
        knee.crossing_s,
        knee.distance_to_knee,
        knee.minimum_hot_error_c,
        knee.crossing_deadline_s,
    );
    eprintln!(
        "[{}] plateau-start decision={:?}",
        variant.name, plateau_row.gpu
    );
    if variant.name == "sim4-cold" {
        let hot_index = post
            .iter()
            .position(|row| (row.time_s - knee.hot_start_s).abs() < 0.1)
            .expect("measured hot start is in the trace");
        assert!(
            hot_index > 0 && hot_index + 1 < post.len(),
            "[sim4-cold] handover neighbors"
        );
        let before = &post[hot_index - 1];
        let handover = &post[hot_index];
        assert!(
            before.gpu.selected == TelemetrySelected::Shadow
                && handover.gpu.selected == TelemetrySelected::Thermal,
            "[sim4-cold] measured handover at t={}: expected Shadow->Thermal, got {:?}->{:?}",
            handover.time_s,
            before.gpu.selected,
            handover.gpu.selected
        );
        let handovers = post[..=hot_index]
            .windows(2)
            .filter(|pair| {
                pair[0].gpu.selected == TelemetrySelected::Shadow
                    && pair[1].gpu.selected == TelemetrySelected::Thermal
            })
            .count();
        assert_eq!(
            handovers, 1,
            "[sim4-cold] exactly one measured thermal handover before the downward-knee timing origin"
        );
        let next = &post[hot_index + 1];
        let error = handover.gpu.err_c.expect("handover error");
        let next_error = next.gpu.err_c.expect("post-handover error");
        let dt = next.time_s - handover.time_s;
        let expected_delta = 2.1 * (next_error - error) + 2.1 / 15.0 * next_error * dt;
        let measured_delta = next.gpu.thermal - handover.gpu.thermal;
        assert!(
            (measured_delta - expected_delta).abs() <= 1.1,
            "[sim4-cold] e_prev/P-response bar after handover t={}: measured delta={measured_delta:.3}, expected={expected_delta:.3}",
            handover.time_s
        );
    }
    assert!(
        knee.crossing_s <= knee.crossing_deadline_s + 1.0,
        "[{}] knee-crossing bound from validated plateau t={:.0}: crossing {:.0} > {:.0}; D={:.0}, e_min={:.3}",
        variant.name,
        knee.plateau_start_s,
        knee.crossing_s,
        knee.crossing_deadline_s,
        knee.distance_to_knee,
        knee.minimum_hot_error_c
    );
    let settling_origin = knee.crossing_s + 90.0;
    let settled_at = first_sustained_in_band(&temp_errors, settling_origin, 1.0, 60.0, 1.0);
    let settling_deadline = settling_origin + 3.0 * 270.0;
    assert!(
        settled_at.is_some_and(|time| time <= settling_deadline),
        "[{}] post-crossing settle bar: origin={settling_origin:.0}, deadline={settling_deadline:.0}, settled={settled_at:?}",
        variant.name
    );
    if let Some(settled_at) = settled_at {
        assert!(
            temp_errors
                .iter()
                .filter(|(time, _)| *time >= settled_at)
                .all(|(_, error)| error.abs() <= 1.0),
            "[{}] permanent post-crossing +/-1C settling bar from t={settled_at:.0} through trace end t={:.0}",
            variant.name,
            post.last().unwrap().time_s,
        );
    }
    if !variant.shadow_enabled || variant.draw_missing_until_s.is_some() {
        let full_thermal_travel = (hot_start_row.gpu.thermal - 2143.0).max(0.0);
        assert!(
            full_thermal_travel >= 900.0,
            "[{}] full-947MHz thermal-only travel bar at hot response t={:.0}: measured travel={full_thermal_travel:.0}MHz; plateau D={:.0}MHz",
            variant.name,
            knee.hot_start_s,
            knee.distance_to_knee
        );
    }
    assert_write_slew(variant.name, post);
    assert_no_relay(variant.name, post, None);

    if let Some(return_at) = variant.draw_missing_until_s {
        for event in [60_u64, return_at] {
            let before = &trace.rows[(event - 1) as usize];
            let after = &trace.rows[event as usize];
            let jump = (after.gpu_cap_mhz - before.gpu_cap_mhz).abs();
            assert!(
                jump <= 300.0 + 1.0,
                "[{}] no-jump bar at missing-draw dwell/return t={event}: {jump:.0}MHz",
                variant.name
            );
        }
    }
    if let Some((start, reached, origin)) = upward_bar {
        assert!(
            reached.is_some_and(|time| time - origin <= 8.0),
            "[{}] applied 1000MHz upward-shadow-recovery <=8s bar from t={origin}: start={start:.0}, reached={reached:?}",
            variant.name
        );

        let hot_index = post
            .iter()
            .position(|row| (row.time_s - knee.hot_start_s).abs() < 0.1)
            .expect("measured hot start is in the trace");
        let shadow_recovery = &post[..hot_index];
        assert!(
            shadow_recovery.iter().all(|row| {
                row.gpu.selected != TelemetrySelected::Shadow
                    || row.gpu.cap <= row.gpu_clock_mhz + 300.0 + 1.0
            }),
            "[{}] every shadow request <= reported clock + 300MHz headroom from t={origin} to handover t={:.0}",
            variant.name,
            knee.hot_start_s,
        );
        let saturated: Vec<_> = shadow_recovery
            .iter()
            .filter(|row| {
                row.gpu.selected == TelemetrySelected::Shadow
                    && (row.gpu_clock_mhz - 2520.0).abs() <= 1.0
            })
            .collect();
        assert!(
            !saturated.is_empty()
                && saturated.iter().any(|row| row.gpu.cap >= 2819.0)
                && saturated.iter().all(|row| row.gpu.cap <= 2821.0),
            "[{}] saturated-reported-clock no-walk bar: expected requests to settle at 2520+300MHz without walking to max; samples={}, range={:.0}..{:.0}",
            variant.name,
            saturated.len(),
            saturated
                .iter()
                .map(|row| row.gpu.cap)
                .fold(f64::INFINITY, f64::min),
            saturated
                .iter()
                .map(|row| row.gpu.cap)
                .fold(f64::NEG_INFINITY, f64::max),
        );
    }
    assert!(
        fan_crest <= target_rpm + 250.0,
        "[{}] fan-crest bar from step t={}: crest={fan_crest:.1} > {:.1}",
        variant.name,
        variant.step_at_s,
        target_rpm + 250.0
    );
}

fn run_gpu_step_matrix() {
    let mut failures = Vec::new();
    for variant in [
        StepVariant {
            name: "sim4-cold",
            step_at_s: 31,
            total_s: 5200,
            warm: false,
            shadow_enabled: true,
            draw_missing_until_s: None,
        },
        StepVariant {
            name: "sim4-warm",
            step_at_s: 6001,
            total_s: 9200,
            warm: true,
            shadow_enabled: true,
            draw_missing_until_s: None,
        },
        StepVariant {
            name: "sim4-shadow-disabled",
            step_at_s: 31,
            total_s: 6200,
            warm: false,
            shadow_enabled: false,
            draw_missing_until_s: None,
        },
        StepVariant {
            name: "sim4-draw-missing",
            step_at_s: 91,
            total_s: 6200,
            warm: false,
            shadow_enabled: true,
            draw_missing_until_s: Some(3000),
        },
    ] {
        if let Err(payload) = std::panic::catch_unwind(|| run_one_gpu_step(variant)) {
            let message = payload
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| {
                    payload
                        .downcast_ref::<&str>()
                        .map(|message| (*message).to_owned())
                })
                .unwrap_or_else(|| "non-string panic".to_owned());
            failures.push(format!("{}: {message}", variant.name));
        }
    }
    assert!(
        failures.is_empty(),
        "sim4 acceptance failures:\n{}",
        failures.join("\n")
    );
}

fn loaded_script(
    status: &ControlStatus,
    config: &Config,
    cpu_load: f64,
    gpu_load: f64,
) -> TickScript {
    TickScript {
        cpu_cap_w: status.cpu_limit_w.unwrap_or(config.cpu_floor_w),
        cpu_demand_frac: cpu_load,
        cpu_util_pct: 100.0 * cpu_load,
        gpu_lock_mhz: Some(status.gpu_max_mhz.unwrap_or(config.gpu_floor_mhz).into()),
        gpu_load_level: Some(gpu_load),
        gpu_powered: Some(true),
        gpu_util_pct: 100.0 * gpu_load,
        cpu_tctl_c: Some(70.0),
        gpu_temp_c: Some(70.0),
        ..TickScript::default()
    }
}

fn first_cap_return(
    rows: &[TraceRow],
    origin_s: f64,
    cpu_target_w: f64,
    gpu_target_mhz: f64,
) -> (Option<f64>, Option<f64>) {
    let cpu = rows
        .iter()
        .find(|row| {
            row.time_s >= origin_s && (row.cpu_cap_w - cpu_target_w).abs() <= 0.1 * cpu_target_w
        })
        .map(|row| row.time_s);
    let gpu = rows
        .iter()
        .find(|row| {
            row.time_s >= origin_s
                && (row.gpu_cap_mhz - gpu_target_mhz).abs() <= 0.1 * gpu_target_mhz
        })
        .map(|row| row.time_s);
    (cpu, gpu)
}

fn run_square_wave(half_cycle_s: u64) {
    const SQUARE_AMBIENT_C: f64 = 56.0;
    let forcing_period_s = 2 * half_cycle_s;
    let forcing_start = WARMUP_S;
    let scored_phase_origin = forcing_start + 1;
    let reference_phase_origin = scored_phase_origin - half_cycle_s / 2;
    let score_start = forcing_start + CONVERGED_WINDOW_S;
    let total = score_start + CONVERGED_WINDOW_S;
    let tag = format!("sim10-square-{half_cycle_s}s-half-cycle");
    let run = |run_tag: &str, phase_origin: u64| {
        run_profile(
            run_tag,
            fixed_gain_config(FAN_TARGET_RPM, true),
            CpuPlantParams::default(),
            GpuPlantParams::nominal(),
            SQUARE_AMBIENT_C,
            total,
            move |tick, status, config| {
                let high =
                    tick < phase_origin || ((tick - phase_origin) / half_cycle_s).is_multiple_of(2);
                let mut script = loaded_script(status, config, 1.0, if high { 1.0 } else { 0.15 });
                script.cpu_pkg_w_override = Some(if high { 54.0 } else { 4.0 });
                script
            },
        )
    };
    let reference = run(
        &format!("{tag}-forcing-reference-shifted"),
        reference_phase_origin,
    );
    let trace = run(&tag, scored_phase_origin);
    assert_ne!(
        reference_phase_origin % forcing_period_s,
        scored_phase_origin % forcing_period_s,
        "[{tag}] forcing-reference provenance requires a nonzero phase shift"
    );
    let window = &trace.rows[score_start as usize..];
    let tstar_range = window
        .iter()
        .filter_map(|row| row.t_star_c)
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), value| {
            (min.min(value), max.max(value))
        });
    assert!(
        tstar_range.1 - tstar_range.0 <= 0.1,
        "[{tag}] square load must run against one fixed live T*: range {:.2}..{:.2}C",
        tstar_range.0,
        tstar_range.1,
    );
    for (name, errors) in [
        (
            "CPU",
            window
                .iter()
                .filter_map(|row| Some(row.cpu.group_c? - row.t_star_c?))
                .collect::<Vec<_>>(),
        ),
        (
            "GPU",
            window
                .iter()
                .filter_map(|row| Some(row.gpu.group_c? - row.t_star_c?))
                .collect::<Vec<_>>(),
        ),
    ] {
        let min = errors.iter().copied().fold(f64::INFINITY, f64::min);
        let max = errors.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        assert!(
            min <= -4.0 && max >= 4.0,
            "[{tag}] controller-row {name} group-error excursion bars: min={min:.2}C must be <=-4C and max={max:.2}C must be >=+4C, half-cycle={half_cycle_s}s/full-period={forcing_period_s}s"
        );
    }
    assert_no_relay(
        &tag,
        &trace.rows,
        Some((
            &reference.rows,
            forcing_period_s as f64,
            scored_phase_origin as f64,
            reference_phase_origin as f64,
            CONVERGED_WINDOW_S as f64,
        )),
    );
}

fn run_curve_target_return() {
    const DOWN_AT: u64 = 5401;
    const RETURN_AT: u64 = 6001;
    let trace = run_profile_control(
        "sim10-curve-return",
        fixed_gain_config(FAN_TARGET_RPM, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        RETURN_AT + 2 * CONVERGED_WINDOW_S,
        |tick, _plant, controller, config| {
            if tick == DOWN_AT {
                controller.on_command(Command::SetFanTarget(2300.0));
            } else if tick == RETURN_AT {
                controller.on_command(Command::SetFanTarget(FAN_TARGET_RPM));
            }
            loaded_script(controller.status(), config, 1.0, 1.0)
        },
    );
    let prior = &trace.rows[(DOWN_AT - 61) as usize..(DOWN_AT - 1) as usize];
    let cpu_prior = prior.iter().map(|row| row.cpu_cap_w).sum::<f64>() / prior.len() as f64;
    let gpu_prior = prior.iter().map(|row| row.gpu_cap_mhz).sum::<f64>() / prior.len() as f64;
    let prior_tstar = prior.last().and_then(|row| row.t_star_c).expect("Curve T*");
    let tstar_return = trace
        .rows
        .iter()
        .find(|row| {
            row.time_s >= RETURN_AT as f64
                && row
                    .t_star_c
                    .is_some_and(|value| (value - prior_tstar).abs() <= 0.1)
        })
        .map(|row| row.time_s)
        .expect("[sim10-curve-return] measured T* return");
    let excursion = &trace.rows[(DOWN_AT - 1) as usize..tstar_return as usize];
    assert!(
        excursion.iter().any(|row| row
            .t_star_c
            .is_some_and(|value| (value - prior_tstar).abs() > 0.1)),
        "[sim10-curve-return] T* must depart before measured return at t={tstar_return:.0}"
    );
    assert!(
        excursion
            .iter()
            .any(|row| (row.cpu_cap_w - cpu_prior).abs() > 0.1 * cpu_prior)
            && excursion
                .iter()
                .any(|row| (row.gpu_cap_mhz - gpu_prior).abs() > 0.1 * gpu_prior),
        "[sim10-curve-return] CPU and GPU caps must each depart the prior 10% band before re-entry; prior={cpu_prior:.2}W/{gpu_prior:.0}MHz"
    );
    let (cpu_return, gpu_return) =
        first_cap_return(&trace.rows, tstar_return, cpu_prior, gpu_prior);
    assert!(
        cpu_return.is_some_and(|time| time <= RETURN_AT as f64 + 450.0),
        "[sim10-curve-return] CPU cap return within 10%/3lambda: origin={RETURN_AT}, returned={cpu_return:?}, prior={cpu_prior:.2}W"
    );
    assert!(
        gpu_return.is_some_and(|time| time <= RETURN_AT as f64 + 810.0),
        "[sim10-curve-return] GPU cap return within 10%/3lambda: origin={RETURN_AT}, returned={gpu_return:?}, prior={gpu_prior:.0}MHz"
    );
    assert_no_relay("sim10-curve-return", &trace.rows, None);
}

fn run_held_target_return() {
    const LOSS_AT: u64 = 5401;
    const RETURN_AT: u64 = 6001;
    let mut prior_tstar_for_driver = None;
    let mut restoring = false;
    let trace = run_profile_control(
        "sim10-held-return",
        fixed_gain_config(FAN_TARGET_RPM, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        RETURN_AT + 5000,
        |tick, _plant, controller, config| {
            if tick == LOSS_AT {
                prior_tstar_for_driver = controller.status().t_star_c;
                controller.on_command(Command::SetFanTarget(2300.0));
            } else if tick == RETURN_AT {
                // The EC-autofan plateau sits above the original request.
                // Apply the equal-and-opposite Held excursion until T*
                // itself reaches its prior value, then restore the original
                // operator target. Cap recovery is timed only from that
                // measured T* crossing below.
                controller.on_command(Command::SetFanTarget(7000.0));
                restoring = true;
            } else if restoring
                && let (Some(prior), Some(current)) =
                    (prior_tstar_for_driver, controller.status().t_star_c)
                && current >= prior
            {
                controller.on_command(Command::SetFanTarget(FAN_TARGET_RPM));
                restoring = false;
            }
            let mut script = loaded_script(controller.status(), config, 1.0, 1.0);
            script.socket_dead = tick >= LOSS_AT;
            script
        },
    );
    let prior = &trace.rows[(LOSS_AT - 61) as usize..(LOSS_AT - 1) as usize];
    let prior_tstar = prior.last().and_then(|row| row.t_star_c).expect("Curve T*");
    let cpu_prior = prior.iter().map(|row| row.cpu_cap_w).sum::<f64>() / prior.len() as f64;
    let gpu_prior = prior.iter().map(|row| row.gpu_cap_mhz).sum::<f64>() / prior.len() as f64;
    assert!(
        trace.rows[(LOSS_AT - 1) as usize..]
            .iter()
            .any(|row| row.tstar_state == Some(TelemetryTStarState::Held)),
        "[sim10-held-return] curve loss must enter Held"
    );
    let tstar_return = trace
        .rows
        .iter()
        .find(|row| {
            row.time_s >= RETURN_AT as f64
                && row
                    .t_star_c
                    .is_some_and(|value| (value - prior_tstar).abs() <= 0.1)
        })
        .map(|row| row.time_s);
    let origin =
        tstar_return.expect("[sim10-held-return] T* must return before cap recovery is timed");
    let excursion = &trace.rows[(LOSS_AT - 1) as usize..origin as usize];
    assert!(
        excursion.iter().any(|row| row
            .t_star_c
            .is_some_and(|value| (value - prior_tstar).abs() > 0.1)),
        "[sim10-held-return] T* must depart before measured return at t={origin:.0}"
    );
    assert!(
        excursion
            .iter()
            .any(|row| (row.cpu_cap_w - cpu_prior).abs() > 0.1 * cpu_prior)
            && excursion
                .iter()
                .any(|row| (row.gpu_cap_mhz - gpu_prior).abs() > 0.1 * gpu_prior),
        "[sim10-held-return] CPU and GPU caps must each depart the prior 10% band before re-entry; prior={cpu_prior:.2}W/{gpu_prior:.0}MHz"
    );
    let (cpu_return, gpu_return) = first_cap_return(&trace.rows, origin, cpu_prior, gpu_prior);
    let deadline = origin + 3.0 * 1440.0;
    assert!(
        cpu_return.is_some_and(|time| time <= deadline)
            && gpu_return.is_some_and(|time| time <= deadline),
        "[sim10-held-return] 3lambda_inner recovery after T* return: origin={origin:.0}, deadline={deadline:.0}, cpu={cpu_return:?}, gpu={gpu_return:?}"
    );
    assert_no_relay("sim10-held-return", &trace.rows, None);
}

fn run_hot_draw_dip_replay() {
    const DIP_AT: u64 = 5401;
    const DIP_END: u64 = DIP_AT + 60;
    let run = |tag: &str, replay_groups: Option<&[(f64, f64)]>| {
        run_profile_control(
            tag,
            fixed_gain_config(FAN_TARGET_RPM, true),
            CpuPlantParams::default(),
            GpuPlantParams::nominal(),
            AMBIENT_C,
            DIP_END + CONVERGED_WINDOW_S,
            |tick, _plant, controller, config| {
                if tick == DIP_AT {
                    controller.on_command(Command::SetFanTarget(2300.0));
                } else if tick == DIP_END {
                    controller.on_command(Command::SetFanTarget(FAN_TARGET_RPM));
                }
                let in_dip = (DIP_AT..DIP_END).contains(&tick);
                let mut script = loaded_script(
                    controller.status(),
                    config,
                    if in_dip && replay_groups.is_none() {
                        0.25
                    } else {
                        1.0
                    },
                    if in_dip && replay_groups.is_none() {
                        0.45
                    } else {
                        1.0
                    },
                );
                if in_dip && let Some(groups) = replay_groups {
                    let (cpu, gpu) = groups[(tick - DIP_AT) as usize];
                    script.cpu_stuck_c = Some(cpu);
                    script.gpu_stuck_c = Some(gpu);
                }
                script
            },
        )
    };
    let dipped = run("sim10-hot-draw-dip", None);
    let groups: Vec<_> = dipped.rows[(DIP_AT - 1) as usize..(DIP_END - 1) as usize]
        .iter()
        .map(|row| {
            (
                row.cpu_raw_c.expect("CPU raw group"),
                row.gpu_raw_c.expect("GPU raw group"),
            )
        })
        .collect();
    let first = &dipped.rows[(DIP_AT - 1) as usize];
    assert!(
        first.cpu.err_c.is_some_and(|error| error < 0.0)
            && first.gpu.err_c.is_some_and(|error| error < 0.0),
        "[sim10-hot-draw-dip] 60s dip origin t={DIP_AT} must begin while each group is hot: cpu={:?}, gpu={:?}",
        first.cpu.err_c,
        first.gpu.err_c
    );
    let replay = run("sim10-equal-error-replay", Some(&groups));
    let dip_end = &dipped.rows[(DIP_END - 2) as usize];
    let replay_end = &replay.rows[(DIP_END - 2) as usize];
    assert!(
        (dip_end.cpu.thermal - replay_end.cpu.thermal).abs() <= 0.51
            && (dip_end.gpu.thermal - replay_end.gpu.thermal).abs() <= 1.1,
        "[sim10-hot-draw-dip] equal-error PI-state bar at t={}: dipped thermal cpu/gpu {:.2}/{:.0}, replay {:.2}/{:.0}",
        DIP_END - 1,
        dip_end.cpu.thermal,
        dip_end.gpu.thermal,
        replay_end.cpu.thermal,
        replay_end.gpu.thermal
    );
    let recovery = &dipped.rows[(DIP_END - 1) as usize..];
    let floor = recovery
        .iter()
        .map(|row| row.gpu_cap_mhz)
        .fold(f64::INFINITY, f64::min);
    let error_origin = recovery
        .iter()
        .position(|row| row.gpu.err_c.is_some_and(|error| error >= 0.0));
    match error_origin {
        None => {
            assert!(
                recovery
                    .iter()
                    .all(|row| row.gpu.err_c.is_some_and(|error| error < 0.0)),
                "[sim10-hot-draw-dip] shadow-recovery inapplicability requires telemetry proving error stayed negative"
            );
            eprintln!(
                "[sim10-hot-draw-dip] shadow-recovery inapplicable: GPU error remained negative after t={DIP_END}"
            );
        }
        Some(origin_index) => {
            let origin_row = &recovery[origin_index];
            let target_cap = floor + 1000.0;
            let reached_index = recovery[origin_index..]
                .iter()
                .position(|row| row.gpu_cap_mhz >= target_cap)
                .map(|offset| origin_index + offset);
            let scored_end = reached_index.unwrap_or(recovery.len() - 1);
            let scored = &recovery[origin_index..=scored_end];
            assert!(
                scored.iter().all(|row| {
                    row.gpu.selected != TelemetrySelected::Shadow
                        || row.gpu.cap <= row.gpu_clock_mhz + 300.0 + 1.0
                }),
                "[sim10-hot-draw-dip] every shadow request <= reported clock + 300MHz headroom from err>=0 origin {:.0} through t={:.0}",
                origin_row.time_s,
                scored.last().unwrap().time_s,
            );
            let purely_shadow = scored
                .iter()
                .all(|row| row.gpu.selected == TelemetrySelected::Shadow);
            if let Some(reached_index) = reached_index.filter(|_| purely_shadow) {
                let reached = &recovery[reached_index];
                assert!(
                    reached.time_s - origin_row.time_s <= 8.0,
                    "[sim10-hot-draw-dip] purely-shadow applied 1000MHz recovery <=8s: err>=0 origin={:.0}, reached={:.0}, floor={floor:.0}, target={target_cap:.0}",
                    origin_row.time_s,
                    reached.time_s
                );
            } else {
                let selection_departed = scored
                    .iter()
                    .any(|row| row.gpu.selected != TelemetrySelected::Shadow);
                let insufficient_runway = target_cap > 3090.0;
                assert!(
                    origin_row.gpu.selected != TelemetrySelected::Shadow
                        || selection_departed
                        || insufficient_runway,
                    "[sim10-hot-draw-dip] shadow-recovery applicability unresolved: nonnegative origin={:.0}, floor={floor:.0}, target={target_cap:.0}, selected={:?}, reached={:?}",
                    origin_row.time_s,
                    origin_row.gpu.selected,
                    reached_index.map(|index| recovery[index].time_s),
                );
                eprintln!(
                    "[sim10-hot-draw-dip] shadow-recovery inapplicable by telemetry: origin={:.0}, selected={:?}, selection_departed={selection_departed}, insufficient_runway={insufficient_runway}",
                    origin_row.time_s, origin_row.gpu.selected,
                );
            }
        }
    }
    assert_no_relay("sim10-hot-draw-dip", &dipped.rows, None);
    assert_no_relay("sim10-equal-error-replay", &replay.rows, None);
}

fn run_recovery_matrix() {
    run_square_wave(60);
    run_square_wave(300);
    run_curve_target_return();
    run_held_target_return();
    run_hot_draw_dip_replay();
}

fn run_bumpless_entry_matrix() {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("wall clock")
        .as_secs();
    let table = crate::fanctrl::table::DutyRpmTable::default();
    let duty = table.duty_for_rpm(FAN_TARGET_RPM);
    let mut persisted = PersistedState {
        lut: Some(gpu_watts_lut()),
        duty_rpm_table: table,
        t_star_last_good: Some(TStarSeed {
            strategy: "quiet16".into(),
            fan_target_rpm: FAN_TARGET_RPM as u32,
            value_c: 68.0,
            saved_at_unix_s: now,
        }),
        ..PersistedState::default()
    };
    persisted.warm_start.insert(
        WarmStart::key("quiet16", duty, true),
        WarmStartEntry {
            cpu_cap_w: 55.0,
            gpu_lock_mhz: 3090,
        },
    );
    const AUTO_AT: u64 = 601;
    let trace = run_profile_control_with_state(
        "sim5-qualified-monitor-auto",
        fixed_gain_config(FAN_TARGET_RPM, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        AUTO_AT + 15,
        persisted,
        AUTO_AT,
        |_tick, _plant, controller, config| {
            let mut s = loaded_script(controller.status(), config, 1.0, 1.0);
            if controller.status().cpu_limit_w.is_none() {
                s.cpu_cap_w = config.cpu_max_w;
            }
            if controller.status().gpu_max_mhz.is_none() {
                s.gpu_lock_mhz = Some(f64::from(config.gpu_max_mhz));
                s.gpu_load_level = Some(1.0);
            }
            s.cpu_stuck_c = Some(65.0);
            s.gpu_stuck_c = Some(65.0);
            s.gpu_temp_c = Some(80.0);
            s
        },
    );
    let monitor = run_profile_control_with_state(
        "sim5-monitor-replay",
        fixed_gain_config(FAN_TARGET_RPM, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        AUTO_AT + 15,
        PersistedState {
            lut: Some(gpu_watts_lut()),
            ..PersistedState::default()
        },
        AUTO_AT + 100,
        |_tick, _plant, controller, config| {
            let mut s = loaded_script(controller.status(), config, 1.0, 1.0);
            s.cpu_cap_w = config.cpu_max_w;
            s.gpu_lock_mhz = Some(f64::from(config.gpu_max_mhz));
            s.gpu_load_level = Some(1.0);
            s.cpu_stuck_c = Some(65.0);
            s.gpu_stuck_c = Some(65.0);
            s.gpu_temp_c = Some(80.0);
            s
        },
    );
    let entry = &trace.rows[..10];
    assert_eq!(
        entry[0].t_star_c,
        Some(68.0),
        "[sim5/qualified-entry] controller did not consume qualified T* seed"
    );
    let cpu_draw_min = entry
        .iter()
        .map(|r| r.cpu_draw_w)
        .fold(f64::INFINITY, f64::min);
    let cpu_draw_max = entry
        .iter()
        .map(|r| r.cpu_draw_w)
        .fold(f64::NEG_INFINITY, f64::max);
    let gpu_clock_min = entry
        .iter()
        .map(|r| r.gpu_clock_mhz)
        .fold(f64::INFINITY, f64::min);
    let gpu_clock_max = entry
        .iter()
        .map(|r| r.gpu_clock_mhz)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(
        cpu_draw_max - cpu_draw_min <= 1.5 && gpu_clock_max - gpu_clock_min <= 30.0,
        "[sim5/qualified-entry] first-10s measured draw/clock spans {:.2}W/{:.1}MHz",
        cpu_draw_max - cpu_draw_min,
        gpu_clock_max - gpu_clock_min
    );
    for (index, row) in entry.iter().enumerate() {
        let cpu_ceiling = row.cpu_guard_max;
        let gpu_ceiling = row.gpu_guard_max;
        assert!(
            row.cpu_cap_w + 0.01 >= (row.cpu_draw_w + 10.0).min(cpu_ceiling),
            "[sim5/qualified-entry] t={} CPU cap {:.2} below active min(draw+headroom,ceiling) {:.2}",
            row.time_s,
            row.cpu_cap_w,
            (row.cpu_draw_w + 10.0).min(cpu_ceiling)
        );
        assert!(
            row.gpu_cap_mhz + 1.0 >= (row.gpu_clock_mhz + 300.0).min(gpu_ceiling),
            "[sim5/qualified-entry] t={} GPU cap {:.0} below active min(clock+headroom,ceiling) {:.0}",
            row.time_s,
            row.gpu_cap_mhz,
            (row.gpu_clock_mhz + 300.0).min(gpu_ceiling)
        );
        let reference_rpm = monitor.samples[(AUTO_AT - 1) as usize + index].1;
        assert!(
            row.rpm + 1.0 >= reference_rpm,
            "[sim5/qualified-entry] Auto boundary dropped fan below identical Monitor replay at t={}: {:.1} < {:.1}RPM",
            row.time_s,
            row.rpm,
            reference_rpm
        );
    }

    let no_draw = run_profile_control_with_state(
        "sim5-no-draw-monitor-auto",
        fixed_gain_config(FAN_TARGET_RPM, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        AUTO_AT + 75,
        PersistedState {
            lut: Some(gpu_watts_lut()),
            ..PersistedState::default()
        },
        AUTO_AT,
        |tick, _plant, controller, config| {
            let mut s = loaded_script(controller.status(), config, 1.0, 1.0);
            if controller.status().cpu_limit_w.is_none() {
                s.cpu_cap_w = config.cpu_max_w;
            }
            if controller.status().gpu_max_mhz.is_none() {
                s.gpu_lock_mhz = Some(f64::from(config.gpu_max_mhz));
                s.gpu_load_level = Some(1.0);
            }
            if tick < AUTO_AT + 65 {
                s.gpu_clock_available = false;
            }
            s.gpu_temp_c = Some(80.0);
            s
        },
    );
    let last_monitor = no_draw.samples[(AUTO_AT - 2) as usize];
    let first_auto = &no_draw.rows[0];
    assert!(
        (first_auto.cpu_cap_w - last_monitor.4).abs() <= 0.01
            && (first_auto.gpu_cap_mhz - last_monitor.5).abs() <= 1.0,
        "[sim5/no-draw-monitor-auto] first Auto caps jumped from the last Monitor applied seed: CPU {:.1}->{:.1}W, GPU {:.0}->{:.0}MHz",
        last_monitor.4,
        first_auto.cpu_cap_w,
        last_monitor.5,
        first_auto.gpu_cap_mhz
    );
    assert_eq!(
        no_draw.samples[(AUTO_AT - 1) as usize].3,
        0.0,
        "[sim5/no-draw-monitor-auto] absent GPU clock unexpectedly supplied a readback"
    );
    let before_dwell = &no_draw.rows[59];
    let after_dwell = &no_draw.rows[60];
    assert!(
        (after_dwell.gpu_cap_mhz - before_dwell.gpu_cap_mhz).abs() <= 105.0,
        "[sim5/no-draw] >60s thermal-only transition exceeded ordinary slew at t={}: {:.0}->{:.0}",
        after_dwell.time_s,
        before_dwell.gpu_cap_mhz,
        after_dwell.gpu_cap_mhz
    );
    assert_eq!(
        after_dwell.gpu.selected,
        TelemetrySelected::Thermal,
        "[sim5/no-draw] selection after dwell"
    );
    assert!(
        no_draw.rows[60..65].iter().all(|r| r.gpu.group_c.is_some()
            && r.gpu.err_c.is_some()
            && r.gpu.thermal.is_finite()
            && r.gpu.selected == TelemetrySelected::Thermal),
        "[sim5/no-draw] group/error/thermal regulation did not remain live through the missing-draw dwell"
    );
    let returned = &no_draw.rows[65];
    assert_eq!(
        returned.gpu.selected,
        TelemetrySelected::Shadow,
        "[sim5/no-draw] valid returned clock did not re-enable measured Shadow"
    );
    assert!(
        (returned.gpu_cap_mhz - no_draw.rows[64].gpu_cap_mhz).abs() <= 105.0,
        "[sim5/no-draw] return exceeded ordinary slew {:.0}->{:.0}",
        no_draw.rows[64].gpu_cap_mhz,
        returned.gpu_cap_mhz
    );
}

fn run_curve_loss_leg(
    tag: &str,
    fan_target_rpm: f64,
    curve: Option<Vec<(f64, u8)>>,
    deadline_s: u64,
    ec_autofan: bool,
) {
    let warmup = 3600;
    let disturbance_clear = warmup + 300;
    let return_at = disturbance_clear + deadline_s + 60;
    let curve_for_trace = curve.clone();
    let trace = run_profile_control(
        tag,
        fixed_gain_config(fan_target_rpm, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        30.0,
        return_at + CONVERGED_WINDOW_S,
        move |tick, plant, controller, config| {
            if tick == 1
                && let Some(points) = curve_for_trace.clone()
            {
                plant
                    .emulator_mut()
                    .edit_curve_in_place(points)
                    .expect("scenario curve");
            }
            if tick == warmup {
                plant.fan_mut().set_offset_rpm(300.0);
                if ec_autofan && let Some(points) = curve_for_trace.as_ref() {
                    plant
                        .emulator_mut()
                        .edit_curve_in_place(
                            points
                                .iter()
                                .map(|(temp, duty)| (*temp, duty.saturating_add(10).min(100)))
                                .collect(),
                        )
                        .expect("temporary EC fan disturbance curve");
                }
            } else if tick == disturbance_clear || tick == return_at {
                plant.fan_mut().set_offset_rpm(0.0);
                if ec_autofan && let Some(points) = curve_for_trace.clone() {
                    plant
                        .emulator_mut()
                        .edit_curve_in_place(points)
                        .expect("restore EC curve while controller view remains stale");
                }
            }
            let load = 1.0;
            let mut script = loaded_script(controller.status(), config, load, load);
            script.force_stale_view = tick >= warmup && tick < return_at;
            script
        },
    );
    let held_counterfactual = run_profile_control(
        &format!("{tag}-held-counterfactual"),
        fixed_gain_config(fan_target_rpm, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        30.0,
        return_at + 30,
        move |tick, plant, controller, config| {
            if tick == 1
                && let Some(points) = curve.clone()
            {
                plant.emulator_mut().edit_curve_in_place(points).unwrap();
            }
            if tick == warmup {
                plant.fan_mut().set_offset_rpm(300.0);
                if ec_autofan && let Some(points) = curve.as_ref() {
                    plant
                        .emulator_mut()
                        .edit_curve_in_place(
                            points
                                .iter()
                                .map(|(temperature, duty)| {
                                    (*temperature, duty.saturating_add(10).min(100))
                                })
                                .collect(),
                        )
                        .unwrap();
                }
            } else if tick == disturbance_clear {
                plant.fan_mut().set_offset_rpm(0.0);
                if ec_autofan && let Some(points) = curve.clone() {
                    plant.emulator_mut().edit_curve_in_place(points).unwrap();
                }
            }
            let mut script = loaded_script(controller.status(), config, 1.0, 1.0);
            script.force_stale_view = tick >= warmup;
            script
        },
    );
    let held = &trace.rows[(warmup - 1) as usize..return_at as usize];
    let converged = &trace.rows[(warmup - 61) as usize..(warmup - 1) as usize];
    let pre_rpm = converged.iter().map(|r| r.rpm).sum::<f64>() / converged.len() as f64;
    let first_half = converged[..30].iter().map(|r| r.rpm).sum::<f64>() / 30.0;
    let last_half = converged[30..].iter().map(|r| r.rpm).sum::<f64>() / 30.0;
    let cpu_cap_span = converged
        .iter()
        .map(|r| r.cpu_cap_w)
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), value| {
            (lo.min(value), hi.max(value))
        });
    let gpu_cap_span = converged
        .iter()
        .map(|r| r.gpu_cap_mhz)
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), value| {
            (lo.min(value), hi.max(value))
        });
    let max_rpm_error = converged
        .iter()
        .map(|r| (r.rpm - r.target_rpm).abs())
        .fold(0.0, f64::max);
    let max_cpu_error = converged
        .iter()
        .filter_map(|r| r.cpu.err_c)
        .map(f64::abs)
        .fold(0.0, f64::max);
    let max_gpu_error = converged
        .iter()
        .filter_map(|r| r.gpu.err_c)
        .map(f64::abs)
        .fold(0.0, f64::max);
    assert!(
        converged
            .iter()
            .all(|r| r.tstar_state == Some(TelemetryTStarState::Curve)
                && (r.rpm - r.target_rpm).abs() <= 150.0
                && r.cpu.err_c.is_some_and(|error| error.abs() <= 1.0)
                && r.gpu.err_c.is_some_and(|error| error.abs() <= 1.0))
            && cpu_cap_span.1 - cpu_cap_span.0 <= 1.0
            && gpu_cap_span.1 - gpu_cap_span.0 <= 105.0
            && (last_half - first_half).abs() <= 150.0,
        "[{tag}] pre-loss Curve state lacked per-sample target proximity, ±1C residency, or cap stability: half-means {:.0}->{:.0}RPM, max errors {:.0}RPM/{:.2}C/{:.2}C, cap spans {:.1}W/{:.0}MHz",
        first_half,
        last_half,
        max_rpm_error,
        max_cpu_error,
        max_gpu_error,
        cpu_cap_span.1 - cpu_cap_span.0,
        gpu_cap_span.1 - gpu_cap_span.0
    );
    assert!(
        held.iter()
            .any(|r| r.tstar_state == Some(TelemetryTStarState::Held)),
        "[{tag}] curve loss never entered Held"
    );
    let origin_tstar = held
        .first()
        .and_then(|r| r.t_star_c)
        .expect("loss origin T*");
    assert!(
        held.iter()
            .any(|r| r.t_star_c.is_some_and(|t| (t - origin_tstar).abs() > 0.1)),
        "[{tag}] Held T* never moved from {origin_tstar:.2}C"
    );
    assert!(
        held.iter()
            .take(30)
            .all(|r| (r.rpm - pre_rpm).abs() > 150.0),
        "[{tag}] injected loss did not produce a sustained 30s departure from converged {:.0}RPM",
        pre_rpm
    );
    let recovery_origin = held[..600.min(held.len())]
        .iter()
        .max_by(|a, b| (a.rpm - pre_rpm).abs().total_cmp(&(b.rpm - pre_rpm).abs()))
        .expect("measured departure apex")
        .time_s;
    let returned = held
        .windows(30)
        .find(|window| {
            window[0].time_s >= recovery_origin
                && window
                    .windows(2)
                    .all(|pair| pair[1].time_s - pair[0].time_s <= 1.0)
                && window.iter().all(|r| (r.rpm - r.target_rpm).abs() <= 150.0)
        })
        .map(|window| window[0].time_s - held[0].time_s);
    let closest = held
        .iter()
        .min_by(|a, b| {
            (a.rpm - a.target_rpm)
                .abs()
                .total_cmp(&(b.rpm - b.target_rpm).abs())
        })
        .unwrap();
    assert!(
        returned.is_some_and(|elapsed| elapsed <= deadline_s as f64),
        "[{tag}] fan did not begin a sustained 30s per-sample return to live target within ±150RPM by 3lambda_eff={deadline_s}s from curve-loss origin {} (measured departure apex {recovery_origin}): {returned:?}; closest t={} rpm/target {:.0}/{:.0}, final {:.0}/{:.0}",
        held[0].time_s,
        closest.time_s,
        closest.rpm,
        closest.target_rpm,
        held.last().unwrap().rpm,
        held.last().unwrap().target_rpm
    );
    assert_no_relay(tag, held, None);
    let return_index = trace
        .rows
        .iter()
        .position(|r| {
            r.time_s >= return_at as f64 && r.tstar_state == Some(TelemetryTStarState::Curve)
        })
        .unwrap_or_else(|| {
            panic!(
                "[{tag}] live curve return absent; tail states={:?}",
                trace.rows[return_at as usize..]
                    .iter()
                    .map(|r| r.tstar_state)
                    .take(40)
                    .collect::<Vec<_>>()
            )
        });
    let before = &trace.rows[return_index - 1];
    let after = &trace.rows[return_index];
    let counterfactual = held_counterfactual
        .rows
        .iter()
        .find(|row| row.time_s == after.time_s)
        .expect("same-time Held counterfactual at Curve return");
    assert!(
        (after.cpu_cap_w - before.cpu_cap_w).abs() <= 0.5
            && (after.gpu_cap_mhz - before.gpu_cap_mhz).abs() <= 105.0
            && (after.cpu_cap_w - counterfactual.cpu_cap_w).abs() <= 0.01
            && (after.gpu_cap_mhz - counterfactual.gpu_cap_mhz).abs() <= 1.0,
        "[{tag}] Curve return diverged from adjacent/Held counterfactual: CPU steps {:.2}/{:.2}W, GPU {:.0}/{:.0}MHz",
        after.cpu_cap_w - before.cpu_cap_w,
        after.cpu_cap_w - counterfactual.cpu_cap_w,
        after.gpu_cap_mhz - before.gpu_cap_mhz,
        after.gpu_cap_mhz - counterfactual.gpu_cap_mhz
    );
}

fn held_schedule_for(points: &[(f64, u8)], fan_target_rpm: f64) -> f64 {
    let table = crate::fanctrl::table::DutyRpmTable::default();
    let duty = table.duty_for_rpm(fan_target_rpm);
    let curve = crate::fanctrl::curve::Curve::from_points(points.to_vec()).expect("test curve");
    let tread = curve.nearest_tread(duty).expect("resolved test tread");
    let target = curve.t_star(tread).expect("test T*");
    (1.0 / curve.slope_at(target).max(1.0)).clamp(0.25, 1.0)
}

fn run_curve_loss_and_restart_matrix() {
    let held_lambda_s = 1440.0;
    let schedule_one_deadline_s = (3.0 * held_lambda_s / 1.0) as u64;
    let schedule_quarter_deadline_s = (3.0 * held_lambda_s / 0.25) as u64;
    const HELD_FAN_TARGET_RPM: f64 = 3950.0;
    const QUIET_HELD_TARGET_RPM: f64 = 2574.0;
    assert_eq!(
        held_schedule_for(QUIET16_POINTS, QUIET_HELD_TARGET_RPM),
        1.0,
        "[sim6/quiet16] resolved local slope must select schedule 1"
    );
    run_curve_loss_leg(
        "sim6-quiet16-schedule1",
        QUIET_HELD_TARGET_RPM,
        None,
        schedule_one_deadline_s,
        false,
    );
    let table = crate::fanctrl::table::DutyRpmTable::default();
    assert_eq!(table.duty_for_rpm(HELD_FAN_TARGET_RPM), 48);
    let near_zero = vec![(0.0, 47), (70.0, 47), (80.0, 50), (110.0, 50)];
    assert_eq!(
        held_schedule_for(&near_zero, HELD_FAN_TARGET_RPM),
        1.0,
        "[sim6/ec-autofan] resolved near-zero curve must select schedule 1"
    );
    run_curve_loss_leg(
        "sim6-ec-autofan-near-zero-schedule1",
        HELD_FAN_TARGET_RPM,
        Some(near_zero),
        schedule_one_deadline_s,
        true,
    );
    let steep = vec![(0.0, 0), (60.0, 0), (80.0, 80), (110.0, 100)];
    assert_eq!(
        held_schedule_for(&steep, HELD_FAN_TARGET_RPM),
        0.25,
        "[sim6/steep] 4%-per-C curve must select schedule 0.25"
    );
    run_curve_loss_leg(
        "sim6-steep-schedule-quarter",
        HELD_FAN_TARGET_RPM,
        Some(steep),
        schedule_quarter_deadline_s,
        true,
    );

    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("wall clock")
        .as_secs();
    let matching = TStarSeed {
        strategy: "quiet16".into(),
        fan_target_rpm: 3100,
        value_c: 84.0,
        saved_at_unix_s: now - 10,
    };
    let mut expired = matching.clone();
    expired.saved_at_unix_s = now - 6 * 3600 - 1;
    let mut future = matching.clone();
    future.saved_at_unix_s = now + 3600;
    let mut wrong_strategy = matching.clone();
    wrong_strategy.strategy = "cool16".into();
    let mut wrong_target = matching.clone();
    wrong_target.fan_target_rpm = 3200;
    let variants = [
        ("matching", Some(matching), Some(84.0)),
        ("wrong-strategy", Some(wrong_strategy), Some(75.0)),
        ("wrong-target", Some(wrong_target), Some(75.0)),
        ("expired", Some(expired), Some(75.0)),
        ("future", Some(future), Some(75.0)),
        ("legacy", None, Some(75.0)),
    ];
    for (name, seed, expected_tstar) in variants {
        let tag = format!("sim6-restart-{name}");
        let restarted = run_profile_control_with_state(
            &tag,
            fixed_gain_config(FAN_TARGET_RPM, true),
            CpuPlantParams::default(),
            GpuPlantParams::nominal(),
            AMBIENT_C,
            32,
            PersistedState {
                lut: Some(gpu_watts_lut()),
                t_star_last_good: seed,
                ..PersistedState::default()
            },
            30,
            |tick, plant, controller, config| {
                if tick == 1 {
                    plant
                        .emulator_mut()
                        .edit_curve_in_place(vec![(0.0, 50), (60.0, 50), (110.0, 100)])
                        .expect("valid curve whose minimum tread is above requested duty");
                }
                let mut s = loaded_script(controller.status(), config, 0.5, 0.5);
                s.cpu_stuck_c = Some(50.0);
                s.gpu_stuck_c = Some(70.0);
                s.ambient_c = Some(70.0);
                s.cpu_cap_w = config.cpu_max_w;
                s.gpu_lock_mhz = Some(f64::from(config.gpu_max_mhz));
                s.gpu_load_level = Some(0.5);
                s
            },
        );
        let row = &restarted.rows[0];
        assert_eq!(row.t_star_c, expected_tstar, "[{tag}] first controller T*");
        assert_eq!(
            row.cpu.group_c,
            Some(50.0),
            "[{tag}] CPU first group average used socket argmax"
        );
        assert_eq!(
            row.gpu.group_c,
            Some(70.0),
            "[{tag}] GPU first group average used socket argmax"
        );
        assert!(
            (row.cpu_cap_w - Config::default().cpu_max_w).abs() <= 0.01
                && (row.gpu_cap_mhz - 3090.0).abs() <= 1.0,
            "[{tag}] first controller caps were not safe seeded maxima: {:.1}/{:.0}",
            row.cpu_cap_w,
            row.gpu_cap_mhz
        );
        assert!(
            restarted
                .rows
                .iter()
                .all(|r| r.tstar_state == Some(TelemetryTStarState::Held)),
            "[{tag}] fresh matching metadata with an unresolvable curve was not Held on entry"
        );
    }
}

fn has_device_unreachable(row: &TraceRow, device: TelemetryDeviceName) -> bool {
    row.telemetry_flags.iter().any(|flag| matches!(flag, TelemetryFlag::DeviceUnreachable { device: got, active: true, .. } if *got == device))
}

fn telemetry_device_close(left: &TelemetryDevice, right: &TelemetryDevice) -> bool {
    let option_close = |a: Option<f64>, b: Option<f64>| match (a, b) {
        (Some(a), Some(b)) => (a - b).abs() <= 0.01,
        (None, None) => true,
        _ => false,
    };
    option_close(left.group_c, right.group_c)
        && option_close(left.err_c, right.err_c)
        && (left.thermal - right.thermal).abs() <= 0.01
        && (left.shadow - right.shadow).abs() <= 0.01
        && (left.cap - right.cap).abs() <= 0.01
        && left.selected == right.selected
        && left.hold == right.hold
        && left.gains_source == right.gains_source
}

macro_rules! enum_registry {
    ($key_fn:ident, $expected_fn:ident, $ty:ty, $arg:ident, [$( $pattern:pat => $example:expr => $key:literal ),+ $(,)?]) => {
        fn $key_fn($arg: &$ty) -> &'static str {
            match $arg { $( $pattern => $key, )+ }
        }
        fn $expected_fn() -> Vec<$ty> { vec![$($example),+] }
    };
}

use crate::control::device_loop::{Bound, Hold};
use crate::control::tstar::{Device as TDevice, TStarFlag};

enum_registry!(tstar_flag_key, expected_tstar_flags, TStarFlag, flag, [
    TStarFlag::ArgmaxUncontrollable(_) => TStarFlag::ArgmaxUncontrollable("ambient".into()) => "argmax_uncontrollable",
    TStarFlag::ArgmaxStuck(_) => TStarFlag::ArgmaxStuck("ambient".into()) => "argmax_stuck",
    TStarFlag::EcUnknownLabel(_) => TStarFlag::EcUnknownLabel("unknown".into()) => "ec_unknown_label",
    TStarFlag::EcUncontrollableUnavailable => TStarFlag::EcUncontrollableUnavailable => "ec_uncontrollable_unavailable",
    TStarFlag::TargetUnreachable(Bound::Floor) => TStarFlag::TargetUnreachable(Bound::Floor) => "target_unreachable_floor",
    TStarFlag::TargetUnreachable(Bound::Max) => TStarFlag::TargetUnreachable(Bound::Max) => "target_unreachable_max",
    TStarFlag::SteepCurve => TStarFlag::SteepCurve => "steep_curve",
    TStarFlag::DeviceUnreachable { device: TDevice::Cpu, bound: Bound::Floor } => TStarFlag::DeviceUnreachable { device: TDevice::Cpu, bound: Bound::Floor } => "device_unreachable_cpu_floor",
    TStarFlag::DeviceUnreachable { device: TDevice::Cpu, bound: Bound::Max } => TStarFlag::DeviceUnreachable { device: TDevice::Cpu, bound: Bound::Max } => "device_unreachable_cpu_max",
    TStarFlag::DeviceUnreachable { device: TDevice::Gpu, bound: Bound::Floor } => TStarFlag::DeviceUnreachable { device: TDevice::Gpu, bound: Bound::Floor } => "device_unreachable_gpu_floor",
    TStarFlag::DeviceUnreachable { device: TDevice::Gpu, bound: Bound::Max } => TStarFlag::DeviceUnreachable { device: TDevice::Gpu, bound: Bound::Max } => "device_unreachable_gpu_max",
]);

enum_registry!(telemetry_flag_key, expected_telemetry_flags, TelemetryFlag, flag, [
    TelemetryFlag::ArgmaxUncontrollable { label: _label, active: _active } => TelemetryFlag::ArgmaxUncontrollable { label: "ambient".into(), active: true } => "argmax_uncontrollable",
    TelemetryFlag::ArgmaxStuck { label: _label, active: _active } => TelemetryFlag::ArgmaxStuck { label: "ambient".into(), active: true } => "argmax_stuck",
    TelemetryFlag::EcUnknownLabel { label: _label, active: _active } => TelemetryFlag::EcUnknownLabel { label: "unknown".into(), active: true } => "ec_unknown_label",
    TelemetryFlag::EcImplausible { label: _label, active: _active } => TelemetryFlag::EcImplausible { label: "raw".into(), active: true } => "ec_implausible",
    TelemetryFlag::EcUncontrollableUnavailable { active: _active } => TelemetryFlag::EcUncontrollableUnavailable { active: true } => "ec_uncontrollable_unavailable",
    TelemetryFlag::GroupLost { device: TelemetryDeviceName::Cpu, active: _active } => TelemetryFlag::GroupLost { device: TelemetryDeviceName::Cpu, active: true } => "group_lost_cpu",
    TelemetryFlag::GroupLost { device: TelemetryDeviceName::Gpu, active: _active } => TelemetryFlag::GroupLost { device: TelemetryDeviceName::Gpu, active: true } => "group_lost_gpu",
    TelemetryFlag::DeviceUnreachable { device: TelemetryDeviceName::Cpu, bound: TelemetryBound::Floor, active: _active } => TelemetryFlag::DeviceUnreachable { device: TelemetryDeviceName::Cpu, bound: TelemetryBound::Floor, active: true } => "device_unreachable_cpu_floor",
    TelemetryFlag::DeviceUnreachable { device: TelemetryDeviceName::Cpu, bound: TelemetryBound::Max, active: _active } => TelemetryFlag::DeviceUnreachable { device: TelemetryDeviceName::Cpu, bound: TelemetryBound::Max, active: true } => "device_unreachable_cpu_max",
    TelemetryFlag::DeviceUnreachable { device: TelemetryDeviceName::Gpu, bound: TelemetryBound::Floor, active: _active } => TelemetryFlag::DeviceUnreachable { device: TelemetryDeviceName::Gpu, bound: TelemetryBound::Floor, active: true } => "device_unreachable_gpu_floor",
    TelemetryFlag::DeviceUnreachable { device: TelemetryDeviceName::Gpu, bound: TelemetryBound::Max, active: _active } => TelemetryFlag::DeviceUnreachable { device: TelemetryDeviceName::Gpu, bound: TelemetryBound::Max, active: true } => "device_unreachable_gpu_max",
    TelemetryFlag::TargetUnreachable { bound: TelemetryBound::Floor, active: _active } => TelemetryFlag::TargetUnreachable { bound: TelemetryBound::Floor, active: true } => "target_unreachable_floor",
    TelemetryFlag::TargetUnreachable { bound: TelemetryBound::Max, active: _active } => TelemetryFlag::TargetUnreachable { bound: TelemetryBound::Max, active: true } => "target_unreachable_max",
    TelemetryFlag::SteepCurve { active: _active } => TelemetryFlag::SteepCurve { active: true } => "steep_curve",
    TelemetryFlag::Legacy { flag: _flag, active: _active } => TelemetryFlag::Legacy { flag: "not_calibrated".into(), active: true } => "legacy",
]);

fn telemetry_flag_active(flag: &TelemetryFlag) -> bool {
    match flag {
        TelemetryFlag::ArgmaxUncontrollable { active, .. }
        | TelemetryFlag::ArgmaxStuck { active, .. }
        | TelemetryFlag::EcUnknownLabel { active, .. }
        | TelemetryFlag::EcImplausible { active, .. }
        | TelemetryFlag::EcUncontrollableUnavailable { active }
        | TelemetryFlag::GroupLost { active, .. }
        | TelemetryFlag::DeviceUnreachable { active, .. }
        | TelemetryFlag::TargetUnreachable { active, .. }
        | TelemetryFlag::SteepCurve { active }
        | TelemetryFlag::Legacy { active, .. } => *active,
    }
}

enum_registry!(hold_key, expected_holds, Hold, hold, [
    Hold::None => Hold::None => "none", Hold::Shadow => Hold::Shadow => "shadow",
    Hold::Clamp(Bound::Floor) => Hold::Clamp(Bound::Floor) => "clamp_floor",
    Hold::Clamp(Bound::Max) => Hold::Clamp(Bound::Max) => "clamp_max",
    Hold::ActuatorMismatch => Hold::ActuatorMismatch => "actuator_mismatch",
    Hold::GroupUnavailable => Hold::GroupUnavailable => "group_unavailable",
    Hold::DrawUnavailable => Hold::DrawUnavailable => "draw_unavailable",
    Hold::Bypass => Hold::Bypass => "bypass",
]);

enum_registry!(selected_key, expected_selected, Selected, selected, [
    Selected::Thermal => Selected::Thermal => "thermal", Selected::Shadow => Selected::Shadow => "shadow",
    Selected::Floor => Selected::Floor => "floor", Selected::Max => Selected::Max => "max",
]);

fn telemetry_selected_key(selected: TelemetrySelected) -> &'static str {
    match selected {
        TelemetrySelected::Thermal => "thermal",
        TelemetrySelected::Shadow => "shadow",
        TelemetrySelected::Floor => "floor",
        TelemetrySelected::Max => "max",
    }
}

fn telemetry_hold_key(hold: TelemetryHold) -> &'static str {
    match hold {
        TelemetryHold::None => "none",
        TelemetryHold::Shadow => "shadow",
        TelemetryHold::Clamp {
            bound: TelemetryBound::Floor,
        } => "clamp_floor",
        TelemetryHold::Clamp {
            bound: TelemetryBound::Max,
        } => "clamp_max",
        TelemetryHold::ActuatorMismatch => "actuator_mismatch",
        TelemetryHold::GroupUnavailable => "group_unavailable",
        TelemetryHold::DrawUnavailable => "draw_unavailable",
        TelemetryHold::Bypass => "bypass",
    }
}

enum_registry!(tstar_state_key, expected_tstar_states, TelemetryTStarState, state, [
    TelemetryTStarState::Curve => TelemetryTStarState::Curve => "curve",
    TelemetryTStarState::Held => TelemetryTStarState::Held => "held",
    TelemetryTStarState::Uncontrollable => TelemetryTStarState::Uncontrollable => "uncontrollable",
    TelemetryTStarState::Released => TelemetryTStarState::Released => "released",
]);

fn run_unreachable_gpu_isolation() {
    const STUCK_AT: u64 = 300;
    let trace = run_profile(
        "sim7-gpu-unreachable",
        fixed_gain_config(FAN_TARGET_RPM, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        1200,
        |tick, status, config| {
            let mut s = loaded_script(status, config, 0.55, 0.55);
            if tick >= STUCK_AT {
                s.gpu_stuck_c = Some(105.0);
                s.raw_only_c = Some(150.0);
            }
            s
        },
    );
    let replay = run_profile(
        "sim7-gpu-control-replay",
        fixed_gain_config(FAN_TARGET_RPM, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        1200,
        |tick, status, config| {
            let mut s = loaded_script(status, config, 0.55, 0.55);
            if tick >= STUCK_AT {
                s.raw_only_c = Some(150.0);
            }
            if tick > 1
                && let Some(fault) = trace.rows.get((tick - 2) as usize)
            {
                s.gpu_lock_mhz = Some(fault.gpu_cap_mhz);
                s.gpu_load_level = Some(0.55);
            }
            s
        },
    );
    trace
        .rows
        .iter()
        .position(|r| has_device_unreachable(r, TelemetryDeviceName::Gpu))
        .expect("[sim7] real GPU DeviceUnreachable flag");
    for (fault, control) in trace.rows[(STUCK_AT - 1) as usize..]
        .iter()
        .zip(&replay.rows[(STUCK_AT - 1) as usize..])
    {
        assert!(
            telemetry_device_close(&fault.cpu, &control.cpu),
            "[sim7] GPU fault changed CPU telemetry decision at t={}: fault={:?}, replay={:?}",
            fault.time_s,
            fault.cpu,
            control.cpu
        );
        assert_eq!(
            fault.cpu.group_c, control.cpu.group_c,
            "[sim7] GPU-unreachable fault changed the CPU group input at t={}",
            fault.time_s
        );
        assert_eq!(
            fault.cpu_cap_w, control.cpu_cap_w,
            "[sim7] GPU fault changed CPU applied cap at t={}",
            fault.time_s
        );
    }
    assert!(
        !trace
            .rows
            .iter()
            .any(|r| has_device_unreachable(r, TelemetryDeviceName::Cpu)),
        "[sim7] GPU fault falsely raised CPU DeviceUnreachable"
    );
}

fn run_behavioral_smoke() {
    let trace = run_profile_control(
        "sim8-controller-flags",
        fixed_gain_config(FAN_TARGET_RPM, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        900,
        |tick, _plant, controller, config| {
            if tick == 850 {
                controller.on_command(Command::SetAuto(false));
            }
            let mut s = loaded_script(controller.status(), config, 0.6, 0.6);
            if tick == 20 {
                s.resumed = true;
            }
            match tick {
                40..=80 => s.unknown_c = Some(100.0),
                100..=180 => {
                    s.ambient_c = Some(100.0);
                    s.charger_c = Some(99.0)
                }
                200..=280 => s.gpu_powered = Some(false),
                300..=370 => s.gpu_group_present = false,
                400..=470 => s.gpu_clock_available = false,
                _ => {}
            }
            s
        },
    );
    let states: Vec<_> = trace.rows.iter().filter_map(|r| r.tstar_state).collect();
    for expected in [
        TelemetryTStarState::Curve,
        TelemetryTStarState::Held,
        TelemetryTStarState::Uncontrollable,
    ] {
        assert!(
            states.contains(&expected),
            "[sim8/tstar] missing {expected:?}; observed={states:?}"
        );
    }
    let state_keys = trace
        .statuses
        .iter()
        .filter_map(|status| status.tstar_state)
        .map(|state| tstar_state_key(&state))
        .collect::<Vec<_>>();
    for expected in ["curve", "held", "uncontrollable"] {
        assert!(
            state_keys.contains(&expected),
            "[sim8/tstar-registry] missing {expected}"
        );
    }
    use crate::control::device_loop::{Bound, Hold};
    use crate::control::tstar::{
        Device as TDevice, EntrySeed, HeldDeviceInput, SensorClass, SensorReading, TStarInput,
        TStarSource,
    };
    let base_input = || TStarInput {
        fresh_view: true,
        replica_reconciled: true,
        curve_points: Some(QUIET16_POINTS.to_vec()),
        snapped_duty: Some(31),
        strategy: Some("quiet16".into()),
        sensors: vec![
            SensorReading {
                label: "ambient_f75303@4d".into(),
                value_c: 25.0,
                class: SensorClass::KnownUncontrollable,
            },
            SensorReading {
                label: "cpu@4c".into(),
                value_c: 70.0,
                class: SensorClass::Controllable,
            },
        ],
        argmax_label: Some("cpu@4c".into()),
        cpu_group_c: Some(70.0),
        gpu_group_c: Some(60.0),
        ..TStarInput::default()
    };
    let mut source_outputs = Vec::new();
    let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
    let mut input = base_input();
    for _ in 0..17 {
        source_outputs.push(source.tick(&input));
    }
    input.fresh_view = false;
    source_outputs.push(source.tick(&input));
    input.ec_valid = false;
    source_outputs.push(source.tick(&input));
    let mut uncontrollable = base_input();
    uncontrollable.argmax_label = Some("ambient_f75303@4d".into());
    uncontrollable.argmax_lead_c = 2.0;
    source_outputs.push(TStarSource::new(EntrySeed::Fallback(70.0)).tick(&uncontrollable));
    let mut unknown = base_input();
    unknown.sensors.push(SensorReading {
        label: "mystery".into(),
        value_c: 90.0,
        class: SensorClass::Unknown,
    });
    unknown.argmax_label = Some("mystery".into());
    let mut unknown_source = TStarSource::new(EntrySeed::Fallback(70.0));
    for _ in 0..17 {
        source_outputs.push(unknown_source.tick(&unknown));
    }
    let mut stuck = base_input();
    stuck.argmax_label = Some("ambient_f75303@4d".into());
    stuck.argmax_lead_c = 2.0;
    stuck.sensors[0].value_c = 80.0;
    let mut stuck_source = TStarSource::new(EntrySeed::Fallback(70.0));
    for _ in 0..318 {
        source_outputs.push(stuck_source.tick(&stuck));
    }
    let mut high = base_input();
    high.fresh_view = false;
    high.curve_points = None;
    high.sensors[0].value_c = 90.0;
    high.cpu_hot_c = 82.0;
    high.gpu_hot_c = 88.0;
    source_outputs.push(TStarSource::new(EntrySeed::Fallback(70.0)).tick(&high));
    let mut low = base_input();
    low.curve_points = Some(vec![(60.0, 20), (80.0, 40)]);
    low.snapped_duty = Some(10);
    source_outputs.push(TStarSource::new(EntrySeed::Fallback(70.0)).tick(&low));
    for (device, bound, group) in [
        (TDevice::Cpu, Bound::Floor, 80.0),
        (TDevice::Cpu, Bound::Max, 60.0),
        (TDevice::Gpu, Bound::Floor, 80.0),
        (TDevice::Gpu, Bound::Max, 60.0),
    ] {
        let mut bounded = base_input();
        bounded.fresh_view = false;
        bounded.dt_s = 5.0;
        bounded.curve_points = Some(vec![(60.0, 10), (70.0, 40), (80.0, 70)]);
        bounded.previous_holds = vec![HeldDeviceInput::new(
            device,
            Hold::Clamp(bound),
            Some(group),
        )];
        let mut bounded_source = TStarSource::new(EntrySeed::Fallback(70.0));
        for _ in 0..13 {
            source_outputs.push(bounded_source.tick(&bounded));
        }
    }
    let source_states = source_outputs
        .iter()
        .map(|out| tstar_state_key(&out.state.into()))
        .collect::<Vec<_>>();
    for key in expected_tstar_states().iter().map(tstar_state_key) {
        assert!(
            source_states.contains(&key),
            "[sim8/tstar-source-state-registry] actual TStarSource output never reached {key}: {source_states:?}"
        );
    }
    let source_flag_keys = source_outputs
        .iter()
        .flat_map(|out| out.flags.iter())
        .map(tstar_flag_key)
        .collect::<Vec<_>>();
    let absent = run_profile(
        "sim8-dgpu-absent-at-entry",
        fixed_gain_config(FAN_TARGET_RPM, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        90,
        |_tick, status, config| {
            let mut s = loaded_script(status, config, 0.6, 0.0);
            s.gpu_powered = Some(false);
            s
        },
    );
    assert!(
        absent
            .rows
            .iter()
            .all(|r| !r.telemetry_flags.iter().any(|f| matches!(
                f,
                TelemetryFlag::GroupLost {
                    device: TelemetryDeviceName::Gpu,
                    ..
                }
            ))),
        "[sim8/absent-dgpu] entry-absent dGPU falsely raised GroupLost"
    );
    assert!(
        trace.rows.iter().any(|r| r
            .telemetry_flags
            .iter()
            .any(|f| matches!(f, TelemetryFlag::EcUnknownLabel { .. }))),
        "[sim8/unknown] unknown argmax flag absent"
    );
    assert!(
        trace
            .rows
            .iter()
            .any(|r| r.telemetry_flags.iter().any(|f| matches!(
                f,
                TelemetryFlag::GroupLost {
                    device: TelemetryDeviceName::Gpu,
                    active: true
                }
            ))),
        "[sim8/group-loss] mid-run GroupLost absent"
    );
    assert!(
        trace
            .rows
            .iter()
            .any(|r| r.gpu.hold == TelemetryHold::DrawUnavailable),
        "[sim8/draw] DrawUnavailable absent"
    );

    let floor_flags = run_profile(
        "sim8-floor-flags",
        fixed_gain_config(FAN_TARGET_RPM, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        900,
        |tick, status, config| {
            let mut s = loaded_script(status, config, 1.0, 1.0);
            if tick <= 650 {
                s.cpu_stuck_c = Some(105.0);
                s.gpu_stuck_c = Some(105.0);
            }
            s
        },
    );
    let mut max_config = fixed_gain_config(FAN_TARGET_RPM, false);
    max_config.shadow_headroom_cpu_w = 100.0;
    let max_flags = run_profile(
        "sim8-max-flags",
        max_config,
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        30.0,
        500,
        |tick, status, config| {
            let mut s = loaded_script(status, config, 1.0, 1.0);
            if tick <= 240 {
                s.cpu_stuck_c = Some(35.0);
                s.gpu_stuck_c = Some(35.0);
            }
            s
        },
    );
    let stuck_flags = run_profile(
        "sim8-stuck-and-implausible",
        fixed_gain_config(FAN_TARGET_RPM, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        520,
        |tick, status, config| {
            let mut s = loaded_script(status, config, 0.6, 0.6);
            if tick <= 360 {
                s.ambient_c = Some(105.0);
                s.raw_only_c = Some(150.0);
            }
            s
        },
    );
    let unavailable_flags = run_profile(
        "sim8-uncontrollable-unavailable",
        fixed_gain_config(FAN_TARGET_RPM, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        150,
        |tick, status, config| {
            let mut s = loaded_script(status, config, 0.6, 0.6);
            if tick <= 90 {
                s.ambient_c = Some(-150.0);
                s.charger_c = Some(-150.0);
            }
            s
        },
    );
    let cpu_loss = run_profile(
        "sim8-cpu-loss",
        fixed_gain_config(FAN_TARGET_RPM, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        150,
        |tick, status, config| {
            let mut s = loaded_script(status, config, 0.6, 0.6);
            if (40..=100).contains(&tick) {
                s.cpu_group_present = false
            }
            s
        },
    );
    let steep = run_profile_control(
        "sim8-steep",
        fixed_gain_config(3950.0, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        90,
        |tick, plant, controller, config| {
            if tick == 1 {
                plant
                    .emulator_mut()
                    .edit_curve_in_place(vec![(0.0, 0), (60.0, 0), (80.0, 80), (110.0, 100)])
                    .unwrap();
            } else if tick == 45 {
                plant
                    .emulator_mut()
                    .edit_curve_in_place(QUIET16_POINTS.to_vec())
                    .unwrap();
            }
            loaded_script(controller.status(), config, 0.5, 0.5)
        },
    );
    let target_floor = run_profile_control(
        "sim8-target-floor",
        fixed_gain_config(1000.0, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        90,
        |tick, plant, controller, config| {
            if tick == 1 {
                plant
                    .emulator_mut()
                    .edit_curve_in_place(vec![(0.0, 20), (60.0, 20), (110.0, 100)])
                    .unwrap();
            } else if tick == 45 {
                plant
                    .emulator_mut()
                    .edit_curve_in_place(vec![(0.0, 0), (60.0, 0), (70.0, 15), (110.0, 100)])
                    .unwrap();
            }
            loaded_script(controller.status(), config, 0.5, 0.5)
        },
    );
    let mismatch = run_profile(
        "sim8-controller-mismatch",
        fixed_gain_config(FAN_TARGET_RPM, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        400,
        |_tick, status, config| {
            let mut sample = loaded_script(status, config, 1.0, 1.0);
            sample.gpu_temp_c = Some(89.0);
            sample.gpu_lock_mhz = None;
            sample.gpu_load_level = None;
            sample.gpu_cap_w = 100.0;
            sample.gpu_demand_frac = 1.0;
            sample.gpu_sm_mhz = 3090.0;
            sample
        },
    );
    let emitted = trace
        .rows
        .iter()
        .chain(&floor_flags.rows)
        .chain(&max_flags.rows)
        .chain(&stuck_flags.rows)
        .chain(&unavailable_flags.rows)
        .chain(&cpu_loss.rows)
        .chain(&steep.rows)
        .chain(&target_floor.rows)
        .chain(&mismatch.rows)
        .flat_map(|r| r.telemetry_flags.iter())
        .filter(|flag| telemetry_flag_active(flag))
        .map(telemetry_flag_key)
        .collect::<Vec<_>>();
    let clearing_traces = [
        trace.rows.as_slice(),
        floor_flags.rows.as_slice(),
        max_flags.rows.as_slice(),
        stuck_flags.rows.as_slice(),
        unavailable_flags.rows.as_slice(),
        cpu_loss.rows.as_slice(),
        steep.rows.as_slice(),
        target_floor.rows.as_slice(),
    ];
    let supported_dynamic_clear_keys = expected_telemetry_flags()
        .iter()
        .filter(|flag| !matches!(flag, TelemetryFlag::Legacy { flag: _, active: _ }))
        .map(telemetry_flag_key)
        .collect::<Vec<_>>();
    for key in supported_dynamic_clear_keys {
        assert!(
            clearing_traces.iter().any(|rows| {
                rows.iter()
                    .position(|row| {
                        row.telemetry_flags.iter().any(|flag| {
                            telemetry_flag_active(flag) && telemetry_flag_key(flag) == key
                        })
                    })
                    .is_some_and(|active| {
                        rows[active + 1..].iter().any(|row| {
                            !row.telemetry_flags
                                .iter()
                                .any(|flag| telemetry_flag_key(flag) == key)
                        })
                    })
            }),
            "[sim8/telemetry-polarity] no real Controller trace showed active:true then disappearance for {key}"
        );
    }
    let decisions = trace
        .rows
        .iter()
        .chain(&floor_flags.rows)
        .chain(&max_flags.rows)
        .chain(&stuck_flags.rows)
        .chain(&unavailable_flags.rows)
        .chain(&cpu_loss.rows)
        .chain(&steep.rows)
        .chain(&target_floor.rows)
        .chain(&mismatch.rows)
        .flat_map(|row| [&row.cpu, &row.gpu])
        .collect::<Vec<_>>();
    let observed_selected = decisions
        .iter()
        .map(|d| telemetry_selected_key(d.selected))
        .collect::<Vec<_>>();
    for key in expected_selected().iter().map(selected_key) {
        assert!(
            observed_selected.contains(&key),
            "[sim8/selected-registry] controller telemetry never emitted {key}; observed={observed_selected:?}"
        );
    }
    let expected_hold_keys = expected_holds().iter().map(hold_key).collect::<Vec<_>>();
    let observed_holds = decisions
        .iter()
        .map(|d| telemetry_hold_key(d.hold))
        .collect::<Vec<_>>();
    let resumed = trace
        .rows
        .iter()
        .find(|row| row.time_s == 20.0)
        .expect("[sim8/unverifiable] resumed row");
    assert!(
        resumed.cpu.hold != TelemetryHold::ActuatorMismatch
            && resumed.gpu.hold != TelemetryHold::ActuatorMismatch
            && [resumed.cpu_actuator, resumed.gpu_actuator].contains(&ActuatorState::Unverifiable),
        "[sim8/unverifiable] resume did not produce real Unverifiable verdicts: {:?}/{:?}",
        resumed.cpu_actuator,
        resumed.gpu_actuator
    );
    for key in expected_hold_keys {
        assert!(
            observed_holds.contains(&key),
            "[sim8/hold-registry] controller telemetry never emitted {key}; observed={observed_holds:?}"
        );
    }
    for key in expected_tstar_flags().iter().map(tstar_flag_key) {
        assert!(
            source_flag_keys.contains(&key),
            "[sim8/tstar-source-flag-registry] actual TStarSource output never emitted {key}; observed={source_flag_keys:?}"
        );
    }
    for key in expected_telemetry_flags().iter().map(telemetry_flag_key) {
        assert!(
            emitted.contains(&key),
            "[sim8/telemetry-flag-registry] real output never emitted {key}; observed={emitted:?}"
        );
    }
}

fn run_guard_episode_matrix() {
    let assert_compliant = |tag: &str, trace: &ScenarioTrace| {
        for (index, row) in trace.rows.iter().enumerate() {
            assert!(
                row.cpu_mismatch_strikes == 0
                    && row.gpu_mismatch_strikes == 0
                    && !row.cpu_released
                    && !row.gpu_released,
                "[{tag}] compliant trace accumulated verifier state at t={}: strikes={}/{}, released={}/{}",
                row.time_s,
                row.cpu_mismatch_strikes,
                row.gpu_mismatch_strikes,
                row.cpu_released,
                row.gpu_released
            );
            if index == 0 {
                continue;
            }
            let previous = index.checked_sub(1).map(|i| &trace.rows[i]);
            let (_, _, _, _, applied_cpu, applied_gpu) = trace.samples[index];
            assert!(
                (applied_cpu - row.cpu_cap_w).abs() <= 0.01
                    || previous.is_some_and(|prior| (applied_cpu - prior.cpu_cap_w).abs() <= 0.01),
                "[{tag}] CPU applied readback at t={} was neither current nor one-command-lag",
                row.time_s
            );
            assert!(
                (applied_gpu - row.gpu_cap_mhz).abs() <= 1.0
                    || previous.is_some_and(|prior| (applied_gpu - prior.gpu_cap_mhz).abs() <= 1.0),
                "[{tag}] GPU applied readback at t={} was neither current nor one-command-lag",
                row.time_s
            );
        }
    };
    for &(name, group_hot) in &[("group-hot", true), ("group-cool", false)] {
        let tag = format!("sim9-gpu-{name}");
        let config = fixed_gain_config(FAN_TARGET_RPM, true);
        let gpu_hot_c = config.gpu_hot_c;
        let trace = run_profile(
            &tag,
            config.clone(),
            CpuPlantParams::default(),
            GpuPlantParams::nominal(),
            AMBIENT_C,
            7000,
            |tick, status, config| {
                let mut s = loaded_script(status, config, 0.6, 0.6);
                if (3601..=3900).contains(&tick) {
                    s.gpu_temp_c = Some(89.0);
                    s.gpu_stuck_c = Some(if group_hot { 78.0 } else { 60.0 });
                } else {
                    s.gpu_temp_c = Some(80.0);
                }
                s
            },
        );
        let episode = &trace.rows[3600..3900];
        let pre_cap = trace.rows[3599].gpu_cap_mhz;
        let pre_draw = trace.rows[3599].gpu_draw_w;
        let pre_load = trace.rows[3599].gpu_load;
        let pre_group = trace.rows[3599].gpu.group_c.expect("pre-episode GPU group");
        let pre_tstar = trace.rows[3599].t_star_c.expect("pre-episode T*");
        assert!(
            episode
                .iter()
                .any(|r| r.flags.contains(&StatusFlag::GpuHot)),
            "[{tag}] five-minute guard episode did not trip"
        );
        let floor_index = trace
            .rows
            .iter()
            .position(|row| {
                row.time_s >= 3601.0 && row.gpu_guard_max <= f64::from(config.gpu_floor_mhz) + 1.0
            })
            .expect("GPU guard first floor-reaching row");
        let clear = trace.rows[3900..]
            .iter()
            .find(|r| !r.flags.contains(&StatusFlag::GpuHot))
            .map(|r| r.time_s)
            .expect("guard clear time");
        let floor_max = trace.rows[floor_index].gpu_guard_max;
        let temperature_clear_index = trace.rows[3900..]
            .iter()
            .position(|row| row.gpu_temp_c <= gpu_hot_c - 4.0)
            .map(|offset| offset + 3900)
            .expect("GPU temperature-threshold clear");
        let temperature_clear = trace.rows[temperature_clear_index].time_s;
        assert!(
            trace.rows[floor_index..temperature_clear_index]
                .iter()
                .all(|row| row.gpu_temp_c > gpu_hot_c - 4.0
                    && (row.gpu_guard_max - floor_max).abs() <= 1.0),
            "[{tag}] GPU guard max failed continuous floor residency from t={} through the measured temperature-clear boundary t={temperature_clear}",
            trace.rows[floor_index].time_s
        );
        let reopen_started = trace.rows[3900..]
            .iter()
            .find(|row| row.gpu_guard_max > floor_max + 1.0)
            .map(|row| row.time_s)
            .expect("first actual GPU guard-max reopen");
        let ceiling_reopened = trace.rows[3900..]
            .iter()
            .find(|row| row.gpu_guard_max + 1.0 >= pre_cap)
            .map(|row| row.time_s)
            .expect("GPU guard ceiling reopened to pre-episode cap");
        let eligible_index = trace.rows[3900..].windows(30).position(|window| window.iter().all(|r| {
            r.gpu_guard_max + 1.0 >= pre_cap
                && r.gpu_temp_c <= gpu_hot_c - 4.0
                && (r.gpu_load-pre_load).abs() <= 0.01
                && r.gpu.group_c.is_some_and(|group| (group-pre_group).abs() <= 1.0)
                && r.t_star_c.is_some_and(|t| (t-pre_tstar).abs() <= 1.0)
                && !r.flags.contains(&StatusFlag::GpuHot)
        })).map(|i| i+3900).unwrap_or_else(|| { let r=trace.rows.last().unwrap(); panic!("[sim9/gpu] sustained full recovery eligibility absent: final temp/draw/T*/thermal/hot={:.1}/{:.1}/{:?}/{:.0}/{} vs pre {:.1}/{:.1}/{:.0}", r.gpu_temp_c,r.gpu_draw_w,r.t_star_c,r.gpu.thermal,r.flags.contains(&StatusFlag::GpuHot),pre_draw,pre_tstar,pre_cap) });
        let gate = trace.rows[eligible_index].time_s;
        assert!(
            temperature_clear <= reopen_started
                && reopen_started <= ceiling_reopened
                && ceiling_reopened <= gate
                && clear <= gate,
            "[{tag}] ceiling recovery began before guard clear: clear={clear}, gate={gate}"
        );
        assert!(
            trace.rows[eligible_index..]
                .iter()
                .all(|r| !r.flags.contains(&StatusFlag::GpuHot)
                    && r.gpu_guard_max + 1.0 >= pre_cap),
            "[{tag}] guard re-tripped after full recovery eligibility at {gate}"
        );
        let fan_overshoot = trace.rows[eligible_index..]
            .iter()
            .map(|row| row.rpm - row.target_rpm)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            fan_overshoot <= 150.0,
            "[{tag}] post-episode fan overshoot above the pre-episode level was {fan_overshoot:.1}RPM, exceeding 150RPM"
        );
        assert_compliant(&tag, &trace);
        let fan_recovered = trace.rows[eligible_index..].windows(30).find(|window| window.iter().all(|r| (r.gpu_cap_mhz-pre_cap).abs() <= pre_cap*0.10)).map(|window| window[0].time_s).unwrap_or_else(|| {
            let (min,max)=trace.rows[eligible_index..].iter().map(|r|r.gpu_cap_mhz).fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),v|(lo.min(v),hi.max(v)));
            panic!("[sim9/gpu] sustained applied-cap return within 10% of pre-episode cap absent: pre={pre_cap}, tail range={min}..{max}")
        });
        assert!(
            fan_recovered <= gate + 810.0,
            "[{tag}] sustained fan return {fan_recovered} exceeded 3lambda from full eligibility {gate}"
        );
        assert!(
            trace
                .rows
                .iter()
                .all(|r| !r.flags.contains(&StatusFlag::LimitNotSticking))
                && trace
                    .gpu_calls
                    .iter()
                    .all(|call| !matches!(call, GpuCall::Release)),
            "[{tag}] compliant one-command-lag path accumulated a strike/release"
        );
        eprintln!(
            "[{tag}] guard-clear={clear:.0}s reopen-start={reopen_started:.0}s ceiling-reopened={ceiling_reopened:.0}s recovery-gate={gate:.0}s"
        );
    }
    for &(name, group_hot) in &[("group-hot", true), ("group-cool", false)] {
        let tag = format!("sim9-cpu-{name}");
        let config = fixed_gain_config(FAN_TARGET_RPM, true);
        let cpu_hot_c = config.cpu_hot_c;
        let cpu = run_profile(
            &tag,
            config.clone(),
            CpuPlantParams::default(),
            GpuPlantParams::nominal(),
            AMBIENT_C,
            6500,
            |tick, status, config| {
                let mut s = loaded_script(status, config, 0.7, 0.7);
                s.cpu_tctl_c = Some(
                    if tick == 3301
                        || (3401..=3403).contains(&tick)
                        || (3501..=3800).contains(&tick)
                    {
                        91.0
                    } else {
                        84.0
                    },
                );
                if (3501..=3800).contains(&tick) {
                    s.cpu_stuck_c = Some(if group_hot { 78.0 } else { 60.0 });
                }
                s
            },
        );
        assert!(
            cpu.rows[3300..3399]
                .iter()
                .all(|r| r.cpu.thermal > 15.0 + 0.1),
            "[{tag}/single] one spike tripped/clamped CPU"
        );
        assert!(
            cpu.rows[3402].cpu.thermal < cpu.rows[3399].cpu.thermal,
            "[{tag}/streak] three-sample streak did not start ratchet: {:.2}->{:.2}W",
            cpu.rows[3399].cpu.thermal,
            cpu.rows[3402].cpu.thermal
        );
        assert!(
            cpu.rows[3500..3800]
                .iter()
                .any(|r| r.cpu_guard_max <= config.cpu_floor_w + 0.1),
            "[{tag}/episode] five-minute episode did not ratchet to floor independent of group error"
        );
        let cpu_pre = cpu.rows[3499].cpu_cap_w;
        let pre_load = cpu.rows[3499].cpu_load;
        let pre_group = cpu.rows[3499].cpu.group_c.expect("pre-episode CPU group");
        let pre_tstar = cpu.rows[3499].t_star_c.expect("pre-episode T*");
        let floor_index = cpu
            .rows
            .iter()
            .position(|row| row.time_s >= 3501.0 && row.cpu_guard_max <= config.cpu_floor_w + 0.01)
            .expect("CPU guard first floor-reaching row");
        let floor_max = cpu.rows[floor_index].cpu_guard_max;
        let cpu_temperature_clear_index = cpu.rows[3800..]
            .iter()
            .position(|row| row.cpu_tctl_c <= cpu_hot_c - 5.0)
            .map(|offset| offset + 3800)
            .expect("CPU temperature-threshold clear");
        let cpu_temperature_clear = cpu.rows[cpu_temperature_clear_index].time_s;
        assert!(
            cpu.rows[floor_index..cpu_temperature_clear_index]
                .iter()
                .all(|row| row.cpu_tctl_c > cpu_hot_c - 5.0
                    && (row.cpu_guard_max - floor_max).abs() <= 0.01),
            "[{tag}] CPU guard max failed continuous floor residency from t={} through the measured Tctl-clear boundary t={cpu_temperature_clear}",
            cpu.rows[floor_index].time_s
        );
        let cpu_reopen_started = cpu.rows[3800..]
            .iter()
            .find(|row| row.cpu_guard_max > floor_max + 0.01)
            .map(|row| row.time_s)
            .expect("first actual CPU guard-max reopen");
        let cpu_ceiling_reopened = cpu.rows[3800..]
            .iter()
            .find(|row| row.cpu_guard_max + 0.01 >= cpu_pre)
            .map(|row| row.time_s)
            .expect("CPU guard ceiling reopened to pre-episode cap");
        let eligible_index = cpu.rows[3800..]
            .windows(30)
            .position(|window| {
                window.iter().all(|r| {
                    r.cpu_guard_max + 0.01 >= cpu_pre
                        && r.cpu_tctl_c <= cpu_hot_c - 5.0
                        && (r.cpu_load - pre_load).abs() <= 0.01
                        && r.cpu
                            .group_c
                            .is_some_and(|group| (group - pre_group).abs() <= 1.0)
                        && r.t_star_c.is_some_and(|t| (t - pre_tstar).abs() <= 1.0)
                })
            })
            .map(|i| i + 3800)
            .expect("[sim9/cpu] sustained full recovery eligibility");
        let cpu_gate = cpu.rows[eligible_index].time_s;
        assert!(
            cpu_temperature_clear <= cpu_reopen_started
                && cpu_reopen_started <= cpu_ceiling_reopened
                && cpu_ceiling_reopened <= cpu_gate
                && cpu_temperature_clear <= cpu_gate
                && cpu.rows[eligible_index..]
                    .iter()
                    .all(|r| r.cpu_guard_max + 0.01 >= cpu_pre),
            "[{tag}] CPU guard ordering/retrip failed: clear={cpu_temperature_clear}, reopen-start={cpu_reopen_started}, ceiling={cpu_ceiling_reopened}, gate={cpu_gate}, pre={cpu_pre:.2}, tail-min={:.2}",
            cpu.rows[eligible_index..]
                .iter()
                .map(|row| row.cpu_guard_max)
                .fold(f64::INFINITY, f64::min)
        );
        let fan_overshoot = cpu.rows[eligible_index..]
            .iter()
            .map(|row| row.rpm - row.target_rpm)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            fan_overshoot <= 150.0,
            "[{tag}] post-episode fan overshoot above the pre-episode level was {fan_overshoot:.1}RPM, exceeding 150RPM"
        );
        assert_compliant(&tag, &cpu);
        let fan_recovered = cpu.rows[eligible_index..]
            .windows(30)
            .find(|window| {
                window
                    .iter()
                    .all(|r| (r.cpu_cap_w - cpu_pre).abs() <= cpu_pre * 0.10)
            })
            .map(|window| window[0].time_s)
            .expect("[sim9/cpu] sustained applied-cap return within 10% of pre-episode cap");
        assert!(
            fan_recovered <= cpu_gate + 810.0,
            "[{tag}] sustained fan return {fan_recovered} exceeded 3lambda from full eligibility {cpu_gate}"
        );
        assert!(
            cpu.rows
                .iter()
                .all(|r| !r.flags.contains(&StatusFlag::LimitNotSticking))
                && cpu
                    .gpu_calls
                    .iter()
                    .all(|call| !matches!(call, GpuCall::Release)),
            "[{tag}] compliant one-command-lag path accumulated a strike/release"
        );
        eprintln!(
            "[{tag}] guard-clear={cpu_temperature_clear:.0}s reopen-start={cpu_reopen_started:.0}s ceiling-reopened={cpu_ceiling_reopened:.0}s recovery-gate={cpu_gate:.0}s"
        );
    }

    let rise = run_profile(
        "sim9-controller-cpu-floor-rise",
        fixed_gain_config(FAN_TARGET_RPM, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        800,
        |tick, status, config| {
            let mut script = loaded_script(status, config, 0.7, 0.7);
            if (301..=660).contains(&tick) {
                script.cpu_tctl_c = Some(91.0);
                script.cpu_stuck_c = Some(if tick <= 600 { 105.0 } else { 60.0 });
            } else if tick > 660 {
                script.cpu_tctl_c = Some(84.0);
                script.cpu_stuck_c = Some(60.0);
            }
            script
        },
    );
    let origin = &rise.rows[659];
    assert!(
        origin.cpu_guard_max <= 15.01 && origin.cpu_cap_w <= 15.01,
        "[sim9/cpu-floor-rise] Controller premise was not the applied guard floor: {:?}",
        origin
    );
    let reached_index = rise.rows[660..]
        .iter()
        .position(|row| row.cpu_cap_w >= origin.cpu_cap_w + 0.5)
        .map(|offset| offset + 660)
        .expect("Controller applied first +0.5W rise");
    let premise = &rise.rows[659..=reached_index];
    let e_min = premise
        .iter()
        .filter_map(|row| row.cpu.err_c)
        .fold(f64::INFINITY, f64::min);
    assert!(
        e_min > 0.0
            && premise
                .iter()
                .all(|row| row.cpu.err_c.is_some_and(|error| error >= e_min)),
        "[sim9/cpu-floor-rise] Controller trace lacked sustained positive measured error: e_min={e_min}, origin={:?}, reached={:?}",
        origin,
        rise.rows[reached_index]
    );
    let pi_errors = premise
        .iter()
        .step_by(5)
        .map(|row| row.cpu.err_c.expect("positive PI-sample error"))
        .collect::<Vec<_>>();
    assert!(
        pi_errors.windows(2).all(|pair| pair[1] + 1e-9 >= pair[0]),
        "[sim9/cpu-floor-rise] measured error decreased at PI samples: {pi_errors:?}"
    );
    let gains = default_gains::<W>(MA_INTERVAL);
    let bound = 0.5 / (gains.kc / gains.ti_s * e_min) + 5.0 + 2.0;
    let reached = rise.rows[reached_index].time_s - origin.time_s;
    assert!(
        reached <= bound,
        "[sim9/cpu-floor-rise] constant positive e_min={e_min} first +0.5W missed analytic bound {bound:.2}s: reached={reached:?}"
    );

    let ignoring = run_profile(
        "sim9-controller-ignoring-card",
        fixed_gain_config(2_638.333_333_333_333_5, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        400,
        |_tick, status, config| {
            let mut s = loaded_script(status, config, 1.0, 1.0);
            s.gpu_temp_c = Some(89.0);
            s.gpu_lock_mhz = None;
            s.gpu_load_level = None;
            s.gpu_cap_w = 100.0;
            s.gpu_demand_frac = 1.0;
            s.gpu_sm_mhz = 3090.0;
            s
        },
    );
    assert!(
        ignoring
            .rows
            .iter()
            .any(|r| r.flags.contains(&StatusFlag::LimitNotSticking)
                && r.gpu.hold == TelemetryHold::ActuatorMismatch),
        "[sim9/controller-ignore] real controller verifier never tripped ignoring card"
    );
}

fn run_sensor_fault_matrix() {
    for &(name, start) in &[("startup", 2_u64), ("midrun", 300)] {
        let tag = format!("sim11-gpu105-{name}");
        let mut config = fixed_gain_config(FAN_TARGET_RPM, true);
        config.gpu_max_mhz = 2143;
        let trace = run_profile(
            &tag,
            config.clone(),
            CpuPlantParams::default(),
            GpuPlantParams::nominal(),
            AMBIENT_C,
            start + 1800,
            move |tick, status, config| {
                let mut s = loaded_script(status, config, 1.0, 1.0);
                if tick >= start && tick < start + 700 {
                    s.gpu_stuck_c = Some(105.0);
                    s.raw_only_c = Some(150.0);
                }
                s
            },
        );
        let pre_fault_seed = trace.rows[(start - 2) as usize].gpu_cap_mhz;
        assert!(
            pre_fault_seed.is_finite() && pre_fault_seed > 1000.0,
            "[{tag}] adjacent pre-fault Auto sample did not establish a real applied seed: {pre_fault_seed}"
        );
        assert!(
            trace.rows[(start - 1) as usize..]
                .iter()
                .all(|r| r.cpu.group_c.is_some_and(f64::is_finite)),
            "[{tag}] GPU fault invalidated the independent CPU physical group"
        );
        let floor = trace
            .rows
            .iter()
            .position(|r| r.time_s >= start as f64 && r.gpu_cap_mhz <= 1001.0)
            .expect("[sim11/105] floor time");
        let hot_origin = ((start - 1) as usize..=floor)
            .find(|&index| trace.rows[index].gpu.err_c.is_some_and(|error| error < 0.0))
            .expect("first measured negative GPU group error");
        let e_min = trace.rows[hot_origin..=floor]
            .iter()
            .filter_map(|r| r.gpu.err_c.map(|e| -e))
            .fold(f64::INFINITY, f64::min);
        assert!(
            e_min.is_finite()
                && e_min > 0.0
                && trace.rows[hot_origin..=floor]
                    .iter()
                    .all(|r| r.gpu.err_c.is_some_and(|e| e <= -e_min + 1e-9)),
            "[{tag}] every floor-travel sample must preserve measured hot-error premise e<=-e_min; e_min={e_min}"
        );
        let bound = (pre_fault_seed - 1000.0).max(0.0) / (2.1 / 15.0 * e_min) + 5.0 + 2.0;
        let travel = trace.rows[floor].time_s - start as f64;
        assert!(
            travel <= bound + 1.0,
            "[{tag}] floor travel {travel:.1}s > D/(Kc/Ti*e_min)+PI/write {bound:.1}s (D={:.0}, e_min={e_min:.2})",
            pre_fault_seed - 1000.0
        );
        let flag = trace.rows[floor..]
            .iter()
            .find(|r| has_device_unreachable(r, TelemetryDeviceName::Gpu))
            .expect("[sim11/105] DeviceUnreachable after floor");
        assert!(
            flag.time_s - trace.rows[floor].time_s <= 60.0,
            "[{tag}] DeviceUnreachable dwell {:.0}s >60s",
            flag.time_s - trace.rows[floor].time_s
        );
        let flush_origin = (start + 700 + MA_INTERVAL as u64) as f64;
        let flush_rows = trace
            .rows
            .iter()
            .filter(|r| r.time_s >= (start + 700) as f64 && r.time_s < flush_origin)
            .collect::<Vec<_>>();
        assert_eq!(
            flush_rows.len(),
            MA_INTERVAL as usize,
            "[{tag}] live boxcar flush sample count"
        );
        assert!(
            flush_rows
                .iter()
                .all(|r| r.gpu_raw_c.is_some_and(|raw| raw < 105.0)),
            "[{tag}] boxcar flush was not driven by live plausible raw samples"
        );
        let live_boxcar_mean = flush_rows
            .iter()
            .map(|row| row.gpu_raw_c.expect("plausible GPU raw sample"))
            .sum::<f64>()
            / MA_INTERVAL as f64;
        let flushed_group = trace
            .rows
            .iter()
            .find(|row| row.time_s == flush_origin)
            .and_then(|row| row.gpu.group_c)
            .expect("live group after exact boxcar flush");
        assert!(
            (flushed_group - live_boxcar_mean).abs() <= 0.05,
            "[{tag}] controller group after 60-s live flush {flushed_group:.3} != raw boxcar mean {live_boxcar_mean:.3}"
        );
        let recovered = trace.rows.windows(30).find(|window| {
            window[0].time_s >= flush_origin
                && window.iter().all(|r| {
                    r.gpu_cap_mhz >= 0.9 * pre_fault_seed
                        && r.gpu.err_c.is_some_and(|e| e.abs() <= 1.0)
                })
        });
        let end = trace.rows.last().unwrap();
        assert!(
            recovered.is_some_and(|window| window[0].time_s <= flush_origin + 810.0),
            "[{tag}] post-flush cap/temperature recovery missed 3lambda: origin={flush_origin}, seeded={pre_fault_seed:.0}, recovered={:?}, end cap/error {:.0}/{:?}",
            recovered.map(|window| window[0].time_s),
            end.gpu_cap_mhz,
            end.gpu.err_c
        );
        let replay = run_profile(
            &format!("{tag}-no-gpu-label-fault-replay"),
            config,
            CpuPlantParams::default(),
            GpuPlantParams::nominal(),
            AMBIENT_C,
            start + 1800,
            |tick, status, config| {
                let mut s = loaded_script(status, config, 1.0, 1.0);
                if tick >= start && tick < start + 700 {
                    s.raw_only_c = Some(150.0);
                }
                if tick > 1 {
                    let forced = &trace.rows[(tick - 2) as usize];
                    s.cpu_cap_w = forced.cpu_cap_w;
                    s.gpu_lock_mhz = Some(forced.gpu_cap_mhz);
                }
                s
            },
        );
        for index in (start - 1) as usize..(start + 699) as usize {
            let fault_group = trace.rows[index]
                .cpu
                .group_c
                .expect("fault trace CPU group");
            let replay_group = replay.rows[index]
                .cpu
                .group_c
                .expect("fault-free replay CPU group");
            assert!(
                (fault_group - replay_group).abs() <= 0.01,
                "[{tag}] GPU label fault changed CPU physical group at t={}: {fault_group:.3} vs replay {replay_group:.3}",
                trace.rows[index].time_s
            );
        }
    }

    let raw = run_profile(
        "sim11-raw150",
        fixed_gain_config(FAN_TARGET_RPM, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        360,
        |tick, status, config| {
            let mut s = loaded_script(status, config, 0.6, 0.6);
            if tick >= 120 {
                s.raw_only_c = Some(150.0);
            }
            s
        },
    );
    let raw_replay = run_profile(
        "sim11-raw150-control-replay",
        fixed_gain_config(FAN_TARGET_RPM, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        360,
        |tick, status, config| {
            let mut s = loaded_script(status, config, 0.6, 0.6);
            if tick > 1 {
                let forced = &raw.rows[(tick - 2) as usize];
                s.cpu_cap_w = forced.cpu_cap_w;
                s.gpu_lock_mhz = Some(forced.gpu_cap_mhz);
            }
            s
        },
    );
    let injection = 119;
    assert!(
        (raw.rows[injection].gpu_cap_mhz - raw_replay.rows[injection].gpu_cap_mhz).abs() <= 0.01
            && (raw.rows[injection].cpu_cap_w - raw_replay.rows[injection].cpu_cap_w).abs() <= 0.01
            && raw.rows[injection].gpu.err_c == raw_replay.rows[injection].gpu.err_c
            && raw.rows[injection].cpu.err_c == raw_replay.rows[injection].cpu.err_c,
        "[sim11/raw150] excluded label directly changed injection-sample device error/cap: raw={:?}, replay={:?}",
        raw.rows[injection],
        raw_replay.rows[injection]
    );
    let raw_interval = &raw.rows[injection..];
    let mismatch_times = raw_interval
        .iter()
        .filter(|r| r.flags.contains(&StatusFlag::EcMismatch) || r.reconciliation_mismatch)
        .map(|r| r.time_s)
        .collect::<Vec<_>>();
    assert!(
        mismatch_times.is_empty(),
        "[sim11/raw150] gate caused steady-state EC mismatch at {mismatch_times:?}"
    );
    assert!(
        raw_interval.iter().all(|row| {
            row.reconciliation_input_c == Some(150)
                && row.cpu_raw_c.is_some_and(|value| value <= 110.0)
                && row.gpu_raw_c.is_some_and(|value| value <= 110.0)
                && row.ec_control_max_c.is_some_and(|value| value <= 110)
                && row.ec_argmax.as_deref() != Some("gpu_vr_f75303@4d")
                && row.tstar_state == Some(TelemetryTStarState::Curve)
                && row.cpu.hold != TelemetryHold::Bypass
                && row.gpu.hold != TelemetryHold::Bypass
        }),
        "[sim11/raw150] implausible raw label entered a plausible control group/argmax or forced Held/Bypass"
    );
    let plausible_argmaxes = raw_interval
        .iter()
        .filter_map(|row| row.ec_argmax.as_deref())
        .collect::<std::collections::BTreeSet<_>>();
    assert_eq!(
        plausible_argmaxes,
        std::collections::BTreeSet::from(["gpu_vram_f75303@4d"]),
        "[sim11/raw150] deterministic plausible argmax changed or included the raw-only gpu_vr label"
    );
    let scored_raw_views =
        raw.rows
            .iter()
            .enumerate()
            .filter(|(index, row)| {
                *index >= injection
                    && row.fanctrl_view_changed
                    && row.reconciliation_scored_count
                        == index
                            .checked_sub(1)
                            .map_or(1, |prior| raw.rows[prior].reconciliation_scored_count + 1)
                    && row.reconciliation_ready
                    && row.reconciliation_ever_scored
                    && row.fanctrl_temperature_c == Some(150.0)
                    && row
                        .fanctrl_ma_c
                        .is_some_and(|value| (value - 150.0).abs() <= 0.01)
                    && row
                        .reconciliation_ma_c
                        .is_some_and(|value| (value - 150.0).abs() <= 0.01)
                    && row.telemetry_flags.iter().any(|flag| {
                        matches!(flag, TelemetryFlag::EcImplausible { active: true, .. })
                    })
            })
            .count();
    assert!(
        scored_raw_views >= 4,
        "[sim11/raw150] implausible label not retained across >=4 scored full views: {scored_raw_views}"
    );

    for &(label, start) in &[
        ("ambient", 1_u64),
        ("ambient", 300_u64),
        ("charger", 1_u64),
        ("charger", 300_u64),
    ] {
        let tag = format!("sim11-{label}105");
        let trace = run_profile(
            &tag,
            fixed_gain_config(FAN_TARGET_RPM, true),
            CpuPlantParams::default(),
            GpuPlantParams::nominal(),
            AMBIENT_C,
            start + 800,
            move |tick, status, config| {
                let mut s = loaded_script(status, config, 0.6, 0.6);
                if tick >= start && tick < start + 620 {
                    if label == "ambient" {
                        s.ambient_c = Some(105.0)
                    } else {
                        s.charger_c = Some(105.0)
                    }
                    if tick >= start + 300 {
                        s.gpu_temp_c = Some(89.0);
                    }
                }
                s
            },
        );
        let stuck_index=trace.rows.iter().position(|r|r.telemetry_flags.iter().any(|f|matches!(f,TelemetryFlag::ArgmaxStuck{label:got,active:true} if got.contains(label)))).expect("[sim11/uncontrollable105] ArgmaxStuck");
        let clear_index=(stuck_index+1..trace.rows.len()).find(|&index| !trace.rows[index].telemetry_flags.iter().any(|f|matches!(f,TelemetryFlag::ArgmaxStuck{label:got,active:true} if got.contains(label)))).expect("[sim11/uncontrollable105] ArgmaxStuck disappeared from Controller telemetry");
        let stuck = &trace.rows[stuck_index];
        let cleared = &trace.rows[clear_index];
        let first_scored_view = (start.div_ceil(30) * 30) as f64;
        assert!(
            stuck.time_s <= first_scored_view + 300.0,
            "[{tag}] ArgmaxStuck after first scored view+300s: origin={first_scored_view}, t={}",
            stuck.time_s
        );
        let quarantine = trace
            .rows
            .iter()
            .filter(|r| r.time_s >= stuck.time_s && r.time_s < cleared.time_s)
            .collect::<Vec<_>>();
        assert!(
            !quarantine.is_empty()
                && quarantine
                    .iter()
                    .all(|r| r.tstar_state == Some(TelemetryTStarState::Held)
                        && r.cpu.hold != TelemetryHold::Bypass
                        && r.gpu.hold != TelemetryHold::Bypass),
            "[{tag}] whole quarantine must remain Held/Regulate without Bypass: {} rows, clear={}",
            quarantine.len(),
            cleared.time_s
        );
        assert_eq!(
            cleared.time_s,
            (start + 649) as f64,
            "[{tag}] quarantine clear must follow exactly 30 fresh >0.5C samples from t={}",
            start + 620
        );
        assert!(
            trace.rows.iter().any(|r| {
                r.time_s >= (start + 300) as f64
                    && r.time_s < (start + 620) as f64
                    && r.flags.contains(&StatusFlag::GpuHot)
            }),
            "[{tag}] GPU guard did not remain live during uncontrollable-sensor quarantine"
        );
        assert!(
            trace
                .rows
                .iter()
                .skip_while(|r| r.time_s < cleared.time_s)
                .take(60)
                .any(|r| r.tstar_state == Some(TelemetryTStarState::Curve)),
            "[{tag}] T* timer did not recover after active:false at {}",
            cleared.time_s
        );
    }
    let sentinel = run_profile(
        "sim11-uncontrollable-sentinel",
        fixed_gain_config(FAN_TARGET_RPM, true),
        CpuPlantParams::default(),
        GpuPlantParams::nominal(),
        AMBIENT_C,
        180,
        |tick, status, config| {
            let mut s = loaded_script(status, config, 0.6, 0.6);
            if tick < 120 {
                s.ambient_c = Some(-150.0);
                s.charger_c = Some(-150.0);
                s.force_stale_view = true;
            }
            s
        },
    );
    let active = sentinel
        .rows
        .iter()
        .position(|r| {
            r.telemetry_flags.iter().any(|f| {
                matches!(
                    f,
                    TelemetryFlag::EcUncontrollableUnavailable { active: true }
                )
            })
        })
        .expect("[sim11/sentinel] active unavailable flag");
    let cleared = (active + 1..sentinel.rows.len())
        .find(|&index| {
            !sentinel.rows[index].telemetry_flags.iter().any(|f| {
                matches!(
                    f,
                    TelemetryFlag::EcUncontrollableUnavailable { active: true }
                )
            })
        })
        .expect("[sim11/sentinel] unavailable flag disappeared from Controller telemetry");
    assert!(
        sentinel
            .rows
            .iter()
            .filter(|row| row.time_s < 120.0)
            .all(|row| {
                row.telemetry_flags.iter().any(|flag| {
                    matches!(
                        flag,
                        TelemetryFlag::EcUncontrollableUnavailable { active: true }
                    )
                }) && row.cpu.group_c.is_some_and(f64::is_finite)
                    && row.gpu.group_c.is_some_and(f64::is_finite)
                    && row.t_star_c.is_some_and(f64::is_finite)
                    && row.tstar_state == Some(TelemetryTStarState::Held)
                    && row.cpu.hold != TelemetryHold::Bypass
                    && row.gpu.hold != TelemetryHold::Bypass
            }),
        "[sim11/sentinel] a fault-active sample lacked flag, finite groups/T*, or Held/Regulate state"
    );
    assert!(
        sentinel.rows[cleared].time_s >= 120.0,
        "[sim11/sentinel] unavailable flag disappeared before plausible return: {}",
        sentinel.rows[cleared].time_s
    );
    let collapsed = &sentinel.rows[active..cleared];
    assert!(
        !collapsed.is_empty(),
        "[sim11/sentinel] unavailable interval empty"
    );
    if let Some(row) = collapsed.iter().find(|r| {
        !(r.cpu.group_c.is_some_and(f64::is_finite)
            && r.gpu.group_c.is_some_and(f64::is_finite)
            && r.t_star_c.is_some_and(f64::is_finite)
            && r.tstar_state == Some(TelemetryTStarState::Held)
            && r.cpu.hold != TelemetryHold::Bypass
            && r.gpu.hold != TelemetryHold::Bypass)
    }) {
        panic!(
            "[sim11/sentinel] invalid collapsed row t={}: groups={:?}/{:?}, T*={:?}/{:?}, holds={:?}/{:?}",
            row.time_s,
            row.cpu.group_c,
            row.gpu.group_c,
            row.t_star_c,
            row.tstar_state,
            row.cpu.hold,
            row.gpu.hold
        );
    }
    assert!(
        sentinel.rows[cleared..]
            .iter()
            .take(60)
            .any(|r| r.tstar_state == Some(TelemetryTStarState::Curve)),
        "[sim11/sentinel] normal Curve did not return after clearing transition"
    );
}

#[cfg(test)]
mod tests {
    #[test]
    fn sims_1_to_3_nominal_cpu_gpu_and_both_heavy() {
        super::run_nominal_sims_1_to_3();
    }

    #[test]
    fn sims_1_to_3_cpu_k_tau_theta_plus_minus_fifty_percent() {
        super::run_cpu_robustness_matrix();
    }

    #[test]
    fn sims_1_to_3_gpu_full_crossed_robustness_matrix() {
        super::run_gpu_robustness_matrix();
    }

    #[test]
    fn sim_4_cold_warm_disabled_shadow_and_missing_draw_steps() {
        super::assert_shadow_disable_boundary();
        super::run_gpu_step_matrix();
    }

    #[test]
    fn sim_10_square_waves_curve_held_and_hot_draw_dips() {
        super::run_recovery_matrix();
    }

    #[test]
    fn sim_5_bumpless_heavy_auto_entry_variants() {
        super::run_bumpless_entry_matrix();
    }

    #[test]
    fn sim_6_curve_loss_and_restart_seed_matrix() {
        super::run_curve_loss_and_restart_matrix();
    }

    #[test]
    fn sim_7_unreachable_gpu_isolated_from_cpu() {
        super::run_unreachable_gpu_isolation();
    }

    #[test]
    fn sim_8_enum_derived_behavioral_smoke() {
        super::run_behavioral_smoke();
    }

    #[test]
    fn sim_9_gpu_and_cpu_guard_episode_matrix() {
        super::run_guard_episode_matrix();
    }

    #[test]
    fn sim_11_stuck_implausible_and_uncontrollable_sensor_matrix() {
        super::run_sensor_fault_matrix();
    }
}

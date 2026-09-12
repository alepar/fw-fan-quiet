//! Closed-loop acceptance scenarios for the revision-4 per-device loops.

use std::path::PathBuf;

use crate::actuators::cmd::test_support::FakeRunner;
use crate::actuators::cpu::CpuActuator;
use crate::actuators::gpu::test_support::{FakeGpu, GpuCall};
use crate::actuators::guard::RestoreGuard;
use crate::actuators::smu_module::SmuModule;
use crate::config::Config;
use crate::control::controller::{Command, ControlStatus, Controller, Effect, Mode, StatusFlag};
use crate::control::device_loop::{
    ActuatorState, DeviceLoop, Gains, Mhz, Selected, ThermalMode, TickInput, W, default_gains,
};
use crate::control::lut::ClockWattsLut;
use crate::state::PersistedState;
use crate::test_support::plant::{
    ChainedPlant, CpuPlantParams, GpuPlantParams, TickScript, gpu_full_load_power_w,
};
use crate::types::{TelemetryDevice, TelemetrySelected, TelemetryTStarState};

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
    gpu_clock_mhz: f64,
    cpu_raw_c: Option<f64>,
    gpu_raw_c: Option<f64>,
    cpu_cap_w: f64,
    gpu_cap_mhz: f64,
    flags: Vec<StatusFlag>,
    effects: Vec<Effect>,
}

#[derive(Debug)]
struct ScenarioTrace {
    rows: Vec<TraceRow>,
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

fn build_controller<'a>(
    runner: &'a FakeRunner,
    tag: &str,
    config: Config,
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
    let persisted = PersistedState {
        lut: Some(gpu_watts_lut()),
        duty_rpm_table: Default::default(),
        ..PersistedState::default()
    };
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
    mut script_for: impl FnMut(u64, &mut ChainedPlant, &mut SimController<'_>, &Config) -> TickScript,
) -> ScenarioTrace {
    let runner = FakeRunner::new();
    let (mut controller, gpu_calls) = build_controller(&runner, tag, config.clone());
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
    controller.on_command(Command::SetAuto(true));
    assert_eq!(controller.status().mode, Mode::Auto, "[{tag}] Auto entry");

    let mut rows = Vec::with_capacity(seconds as usize);
    for tick in 0..seconds {
        let script = script_for(tick + 1, &mut plant, &mut controller, &config);
        let sample = plant.tick(&script);
        let cpu_raw_c = sample.ec.as_ref().and_then(|ec| ec.cpu_group_c);
        let gpu_raw_c = sample.ec.as_ref().and_then(|ec| ec.gpu_group_c);
        let effects = controller.on_sample(&sample);
        let status = controller.status();
        let cpu = status.cpu.clone().unwrap_or_else(|| {
            panic!(
                "[{tag}] CPU decision missing at tick {}, mode={:?}, flags={:?}",
                tick + 1,
                status.mode,
                status.flags
            )
        });
        let gpu = status.gpu.clone().unwrap_or_else(|| {
            panic!(
                "[{tag}] GPU decision missing at tick {}, mode={:?}, flags={:?}",
                tick + 1,
                status.mode,
                status.flags
            )
        });
        rows.push(TraceRow {
            time_s: sample.t_mono,
            mode: status.mode,
            rpm: sample.max_fan_rpm(),
            target_rpm: status.fan_target_rpm,
            t_star_c: status.t_star_c,
            tstar_state: status.tstar_state,
            cpu_cap_w: status.cpu_limit_w.unwrap_or(cpu.cap),
            gpu_cap_mhz: status.gpu_max_mhz.map_or(gpu.cap, f64::from),
            cpu,
            gpu,
            cpu_draw_w: sample.cpu_pkg_w,
            gpu_draw_w: sample.gpu_w,
            gpu_clock_mhz: sample.gpu_sm_mhz,
            cpu_raw_c,
            gpu_raw_c,
            flags: status.flags.clone(),
            effects,
        });
    }
    ScenarioTrace {
        rows,
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
}

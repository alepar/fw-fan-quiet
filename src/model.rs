//! TEA-style UI model: single source of UI state, mutated only in update().

use crate::control::{Command, ControlStatus};
// Shared with the controller so display default and echoed status agree.
use crate::control::controller::{DEFAULT_FAN_TARGET_RPM, Mode};
use crate::event::Event;
use crate::ring::Ring;
use crate::types::Sample;
use crossterm::event::{KeyCode, KeyEventKind, KeyModifiers};

/// Ring capacity: 5 minutes of history at 1 Hz.
pub const RING_CAP: usize = 300;

// Manual-mode key steps/seeds/clamps (plan §M2). Local clamps mirror the
// controller's own clamps; the controller's echoed Status stays the truth.
const CPU_STEP_W: f64 = 2.0;
/// Framework Balanced sustained limit: the first press steps from stock.
const CPU_SEED_W: f64 = 40.0;
const CPU_MIN_W: f64 = 10.0;
// The CPU manual-step ceiling is no longer a const: it comes from the
// controller's echoed status (`status.cpu_max_w`, the config operating max) so
// the UI clamps to the same limit the controller/actuator honor.
/// ~14 driver bins per press.
const GPU_STEP_MHZ: u32 = 105;
/// Stock GPU max boost clock.
const GPU_SEED_MHZ: u32 = 3090;
const GPU_MIN_MHZ: u32 = 1000;
const GPU_MAX_MHZ: u32 = 3090;
const FAN_STEP_RPM: f64 = 250.0;
const FAN_MIN_RPM: f64 = 1000.0;
const FAN_MAX_RPM: f64 = 7000.0;
/// Floor steps (Task 29): CPU floor moves in 1 W steps within the manual
/// clamp range; the GPU floor reuses the manual clock step/clamps.
const CPU_FLOOR_STEP_W: f64 = 1.0;

pub struct Model {
    pub max_fan: Ring,
    pub cpu_w: Ring,
    pub gpu_w: Ring,
    pub cpu_temp: Ring,
    pub gpu_temp: Ring,
    pub gpu_mhz: Ring,
    /// Average CPU core clock (MHz); no validity flag exists for it, so a
    /// failed cpufreq read charts as 0 rather than a gap.
    pub cpu_mhz: Ring,
    /// Most recent full sample (status panel).
    pub latest: Option<Sample>,
    /// Latest status from the controller.
    pub status: ControlStatus,
    /// false => main loop exits.
    pub running: bool,
    /// Fan target shown in the header/fan chart; kept in sync with the
    /// controller via commands + echoed Status (the Auto loop consumes it).
    pub fan_target_rpm: f64,
    /// Locally tracked CPU setpoint (W): what the next c/C press steps from.
    /// None until the first press or Status sync; re-seeds after 'p'.
    cpu_setpoint_w: Option<f64>,
    /// Locally tracked GPU max-clock setpoint (MHz); same lifecycle.
    gpu_setpoint_mhz: Option<u32>,
    /// Locally tracked floors (what the next f/F/d/D press steps from);
    /// seeded from the defaults, kept in sync via the echoed Status. Both
    /// travel together in every `Command::SetFloors`.
    cpu_floor_w: f64,
    gpu_floor_mhz: u32,
}

impl Model {
    pub fn new() -> Self {
        let status = ControlStatus::default();
        Self {
            max_fan: Ring::new(RING_CAP),
            cpu_w: Ring::new(RING_CAP),
            gpu_w: Ring::new(RING_CAP),
            cpu_temp: Ring::new(RING_CAP),
            gpu_temp: Ring::new(RING_CAP),
            gpu_mhz: Ring::new(RING_CAP),
            cpu_mhz: Ring::new(RING_CAP),
            latest: None,
            running: true,
            fan_target_rpm: DEFAULT_FAN_TARGET_RPM,
            cpu_setpoint_w: None,
            gpu_setpoint_mhz: None,
            cpu_floor_w: status.cpu_floor_w,
            gpu_floor_mhz: status.gpu_floor_mhz,
            status,
        }
    }

    /// The ONLY place UI state changes (TEA update). Returned commands are
    /// forwarded to the controller by the main loop.
    pub fn update(&mut self, ev: Event) -> Vec<Command> {
        match ev {
            Event::Sample(s) => {
                // Invalid readings become NaN in the rings: the view filters
                // NaN points out, so a lost sensor renders as a gap instead
                // of a misleading dip to 0.
                let nan_unless = |valid: bool, v: f64| if valid { v } else { f64::NAN };
                self.max_fan.push(nan_unless(s.fan_valid, s.max_fan_rpm()));
                self.cpu_w.push(s.cpu_pkg_w);
                self.gpu_w.push(nan_unless(s.gpu_w_valid, s.gpu_w));
                self.cpu_temp
                    .push(nan_unless(s.cpu_temp_valid, s.cpu_temp_c));
                self.gpu_temp
                    .push(nan_unless(s.gpu_temp_valid, s.gpu_temp_c));
                self.gpu_mhz.push(nan_unless(s.gpu_mhz_valid, s.gpu_sm_mhz));
                self.cpu_mhz.push(s.cpu_avg_mhz);
                self.latest = Some(s);
            }
            Event::Input(key) => {
                // Kitty-protocol terminals also deliver Repeat/Release events;
                // only act on presses.
                if key.kind != KeyEventKind::Press {
                    return Vec::new();
                }
                match (key.code, key.modifiers) {
                    // No command: main's shutdown sequence sends Quit itself.
                    (KeyCode::Char('q'), KeyModifiers::NONE) => self.running = false,
                    // Belt and suspenders next to the signal handler.
                    (KeyCode::Char('c'), KeyModifiers::CONTROL) => self.running = false,
                    // Esc only means something while a calibration runs.
                    (KeyCode::Esc, KeyModifiers::NONE) => {
                        if self.status.mode == Mode::Calibrating {
                            return vec![Command::AbortCalibration];
                        }
                    }
                    // Shifted letters arrive as uppercase Char + SHIFT.
                    (KeyCode::Char(ch), m)
                        if m == KeyModifiers::NONE || m == KeyModifiers::SHIFT =>
                    {
                        return self.on_manual_key(ch);
                    }
                    _ => {}
                }
            }
            Event::Status(cs) => {
                // The controller's clamped truth wins over local tracking
                // (it echoes what we sent, so this cannot fight local edits).
                self.cpu_setpoint_w = cs.cpu_limit_w;
                self.gpu_setpoint_mhz = cs.gpu_max_mhz;
                self.fan_target_rpm = cs.fan_target_rpm;
                self.cpu_floor_w = cs.cpu_floor_w;
                self.gpu_floor_mhz = cs.gpu_floor_mhz;
                self.status = cs;
            }
        }
        Vec::new()
    }

    /// Manual-mode keymap: lowercase steps down, uppercase steps up.
    fn on_manual_key(&mut self, ch: char) -> Vec<Command> {
        // Mirror the controller's Calibrating rejection: it refuses these
        // commands while the runner owns actuation, so stepping the LOCAL
        // setpoints/floors/target here would silently drift them away from
        // the applied truth. 'a' and 'k' carry their own mode gates below.
        if self.status.mode == Mode::Calibrating
            && matches!(
                ch,
                'c' | 'C' | 'g' | 'G' | 't' | 'T' | 'f' | 'F' | 'd' | 'D' | 'p'
            )
        {
            return Vec::new();
        }
        match ch {
            'c' | 'C' => {
                let step = if ch == 'C' { CPU_STEP_W } else { -CPU_STEP_W };
                let v = (self.cpu_setpoint_w.unwrap_or(CPU_SEED_W) + step)
                    .clamp(CPU_MIN_W, self.status.cpu_max_w);
                self.cpu_setpoint_w = Some(v);
                vec![Command::SetCpuW(v)]
            }
            'g' | 'G' => {
                let cur = self.gpu_setpoint_mhz.unwrap_or(GPU_SEED_MHZ);
                let stepped = if ch == 'G' {
                    cur.saturating_add(GPU_STEP_MHZ)
                } else {
                    cur.saturating_sub(GPU_STEP_MHZ)
                };
                let v = stepped.clamp(GPU_MIN_MHZ, GPU_MAX_MHZ);
                self.gpu_setpoint_mhz = Some(v);
                vec![Command::SetGpuMaxClock(v)]
            }
            't' | 'T' => {
                let step = if ch == 'T' {
                    FAN_STEP_RPM
                } else {
                    -FAN_STEP_RPM
                };
                let v = (self.fan_target_rpm + step).clamp(FAN_MIN_RPM, FAN_MAX_RPM);
                // Updated locally too for an instant redraw; the command keeps
                // the controller (and the Auto fan target) in sync.
                self.fan_target_rpm = v;
                vec![Command::SetFanTarget(v)]
            }
            'f' | 'F' => {
                // CPU floor: safety config, allowed in any mode (the
                // controller rejects it only while Calibrating).
                let step = if ch == 'F' {
                    CPU_FLOOR_STEP_W
                } else {
                    -CPU_FLOOR_STEP_W
                };
                self.cpu_floor_w =
                    (self.cpu_floor_w + step).clamp(CPU_MIN_W, self.status.cpu_max_w);
                self.set_floors()
            }
            'd' | 'D' => {
                // GPU clock floor, same step size as the manual clock key.
                let stepped = if ch == 'D' {
                    self.gpu_floor_mhz.saturating_add(GPU_STEP_MHZ)
                } else {
                    self.gpu_floor_mhz.saturating_sub(GPU_STEP_MHZ)
                };
                self.gpu_floor_mhz = stepped.clamp(GPU_MIN_MHZ, GPU_MAX_MHZ);
                self.set_floors()
            }
            'p' => {
                // Pause: back to Monitor mode with stock limits, app keeps
                // running. Cleared setpoints make the next press re-seed.
                self.cpu_setpoint_w = None;
                self.gpu_setpoint_mhz = None;
                vec![Command::ReleaseAll]
            }
            'a' => {
                // Auto toggle, driven off the controller's echoed mode (the
                // truth): blocked while a calibration runs (the runner owns
                // actuation; the controller would reject it anyway).
                match self.status.mode {
                    Mode::Calibrating => Vec::new(),
                    Mode::Auto => vec![Command::SetAuto(false)],
                    Mode::Monitor | Mode::Manual => vec![Command::SetAuto(true)],
                }
            }
            'k' => {
                // Monitor starts from the configured floor pair; Auto is
                // suspended in place and resumes from its held pair.
                if matches!(self.status.mode, Mode::Monitor | Mode::Auto) {
                    vec![Command::StartCalibration]
                } else {
                    Vec::new()
                }
            }
            _ => Vec::new(),
        }
    }

    /// Both floors travel together in every SetFloors command, so the
    /// controller's config copy never sees a partial update.
    fn set_floors(&self) -> Vec<Command> {
        vec![Command::SetFloors {
            cpu_w: self.cpu_floor_w,
            gpu_mhz: self.gpu_floor_mhz,
        }]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::Command;
    use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};

    fn key(c: char) -> KeyEvent {
        KeyEvent::new(KeyCode::Char(c), KeyModifiers::NONE)
    }

    #[test]
    fn sample_event_fills_rings() {
        let mut m = Model::new();
        m.update(Event::Sample(Sample {
            fan1_rpm: 2000.0,
            fan_valid: true,
            ..Default::default()
        }));
        assert_eq!(m.max_fan.last(), Some(2000.0));
    }

    #[test]
    fn invalid_readings_land_as_nan() {
        let mut m = Model::new();
        m.update(Event::Sample(Sample {
            fan1_rpm: 2000.0,
            cpu_temp_c: 55.0,
            gpu_w: 20.0,
            gpu_temp_c: 45.0,
            gpu_sm_mhz: 1500.0,
            fan_valid: false,
            cpu_temp_valid: false,
            gpu_w_valid: false,
            gpu_temp_valid: false,
            gpu_mhz_valid: false,
            ..Default::default()
        }));
        assert!(m.max_fan.last().unwrap().is_nan());
        assert!(m.cpu_temp.last().unwrap().is_nan());
        assert!(m.gpu_w.last().unwrap().is_nan());
        assert!(m.gpu_temp.last().unwrap().is_nan());
        assert!(m.gpu_mhz.last().unwrap().is_nan());
    }

    #[test]
    fn q_key_quits() {
        let mut m = Model::new();
        m.update(Event::Input(key('q')));
        assert!(!m.running);
    }

    #[test]
    fn ctrl_c_quits() {
        let mut m = Model::new();
        m.update(Event::Input(KeyEvent::new(
            KeyCode::Char('c'),
            KeyModifiers::CONTROL,
        )));
        assert!(!m.running);
    }

    #[test]
    fn release_q_does_not_quit() {
        let mut m = Model::new();
        m.update(Event::Input(KeyEvent::new_with_kind(
            KeyCode::Char('q'),
            KeyModifiers::NONE,
            crossterm::event::KeyEventKind::Release,
        )));
        assert!(m.running);
    }

    #[test]
    fn other_keys_ignored() {
        let mut m = Model::new();
        m.update(Event::Input(key('x')));
        assert!(m.running);
    }

    #[test]
    fn rings_capped_at_300() {
        let mut m = Model::new();
        for i in 0..400 {
            m.update(Event::Sample(Sample {
                fan1_rpm: i as f64,
                fan2_rpm: i as f64,
                cpu_pkg_w: i as f64,
                gpu_w: i as f64,
                cpu_temp_c: i as f64,
                gpu_temp_c: i as f64,
                gpu_sm_mhz: i as f64,
                ..Default::default()
            }));
        }
        assert_eq!(m.max_fan.len(), 300);
        assert_eq!(m.cpu_w.len(), 300);
        assert_eq!(m.gpu_w.len(), 300);
        assert_eq!(m.cpu_temp.len(), 300);
        assert_eq!(m.gpu_temp.len(), 300);
        assert_eq!(m.gpu_mhz.len(), 300);
        assert_eq!(m.cpu_mhz.len(), 300);
    }

    #[test]
    fn sample_updates_latest() {
        let mut m = Model::new();
        m.update(Event::Sample(Sample {
            cpu_temp_c: 71.5,
            ..Default::default()
        }));
        assert!(m.latest.is_some());
        assert_eq!(m.latest.unwrap().cpu_temp_c, 71.5);
    }

    #[test]
    fn status_event_stored() {
        use crate::control::controller::{Mode, StatusFlag};
        let mut m = Model::new();
        let cs = ControlStatus {
            mode: Mode::Manual,
            cpu_limit_w: Some(20.0),
            gpu_max_mhz: Some(1500),
            fan_target_rpm: 2500.0,
            flags: vec![StatusFlag::Resumed],
            calib: None,
            ..ControlStatus::default()
        };
        m.update(Event::Status(cs.clone()));
        assert_eq!(m.status, cs);
    }

    #[test]
    fn max_fan_ring_takes_higher_of_two_fans() {
        let mut m = Model::new();
        m.update(Event::Sample(Sample {
            fan1_rpm: 1800.0,
            fan2_rpm: 2200.0,
            fan_valid: true,
            ..Default::default()
        }));
        assert_eq!(m.max_fan.last(), Some(2200.0));
    }

    // --- Task 15: manual-mode keys ---

    fn shift_key(c: char) -> KeyEvent {
        KeyEvent::new(KeyCode::Char(c), KeyModifiers::SHIFT)
    }

    #[test]
    fn c_key_seeds_and_steps_cpu() {
        let mut m = Model::new();
        // First press seeds from 40 W (Framework Balanced sustained) - 2 W.
        assert_eq!(
            m.update(Event::Input(key('c'))),
            vec![Command::SetCpuW(38.0)]
        );
        assert_eq!(
            m.update(Event::Input(key('c'))),
            vec![Command::SetCpuW(36.0)]
        );
    }

    #[test]
    fn shift_c_steps_up() {
        let mut m = Model::new();
        m.update(Event::Input(key('c'))); // seed -> 38
        assert_eq!(
            m.update(Event::Input(shift_key('C'))),
            vec![Command::SetCpuW(40.0)]
        );
    }

    #[test]
    fn shift_c_first_press_seeds_then_steps_up() {
        let mut m = Model::new();
        assert_eq!(
            m.update(Event::Input(shift_key('C'))),
            vec![Command::SetCpuW(42.0)] // 40 seed + 2
        );
    }

    #[test]
    fn cpu_clamps_at_bounds_and_stays_idempotent() {
        let mut m = Model::new();
        // Up from the 38 W seed: 8 steps hit the 54 W ceiling.
        m.update(Event::Input(key('c')));
        for _ in 0..7 {
            m.update(Event::Input(shift_key('C')));
        }
        assert_eq!(
            m.update(Event::Input(shift_key('C'))),
            vec![Command::SetCpuW(54.0)]
        );
        // At the bound: repeated presses still emit the bound value.
        assert_eq!(
            m.update(Event::Input(shift_key('C'))),
            vec![Command::SetCpuW(54.0)]
        );
        // All the way down: clamps at the 10 W floor and stays there.
        for _ in 0..21 {
            m.update(Event::Input(key('c')));
        }
        assert_eq!(
            m.update(Event::Input(key('c'))),
            vec![Command::SetCpuW(10.0)]
        );
        assert_eq!(
            m.update(Event::Input(key('c'))),
            vec![Command::SetCpuW(10.0)]
        );
    }

    #[test]
    fn g_key_seeds_and_steps_gpu() {
        let mut m = Model::new();
        // First press seeds from the 3090 MHz stock max - 105 MHz.
        assert_eq!(
            m.update(Event::Input(key('g'))),
            vec![Command::SetGpuMaxClock(2985)]
        );
        // 2985 - 19*105 = 990 < 1000: clamps at the floor and stays there.
        for _ in 0..18 {
            m.update(Event::Input(key('g')));
        }
        assert_eq!(
            m.update(Event::Input(key('g'))),
            vec![Command::SetGpuMaxClock(1000)]
        );
        assert_eq!(
            m.update(Event::Input(key('g'))),
            vec![Command::SetGpuMaxClock(1000)]
        );
    }

    #[test]
    fn shift_g_steps_up_and_clamps_at_stock_max() {
        let mut m = Model::new();
        m.update(Event::Input(key('g'))); // seed -> 2985
        assert_eq!(
            m.update(Event::Input(shift_key('G'))),
            vec![Command::SetGpuMaxClock(3090)]
        );
        assert_eq!(
            m.update(Event::Input(shift_key('G'))),
            vec![Command::SetGpuMaxClock(3090)]
        );
    }

    #[test]
    fn t_keys_adjust_fan_target() {
        let mut m = Model::new();
        // Default 3000 - 250.
        assert_eq!(
            m.update(Event::Input(key('t'))),
            vec![Command::SetFanTarget(2750.0)]
        );
        assert_eq!(m.fan_target_rpm, 2750.0);
        assert_eq!(
            m.update(Event::Input(shift_key('T'))),
            vec![Command::SetFanTarget(3000.0)]
        );
        // Clamp at the 1000 RPM floor.
        for _ in 0..8 {
            m.update(Event::Input(key('t')));
        }
        assert_eq!(
            m.update(Event::Input(key('t'))),
            vec![Command::SetFanTarget(1000.0)]
        );
        assert_eq!(m.fan_target_rpm, 1000.0);
    }

    #[test]
    fn p_releases_and_clears() {
        let mut m = Model::new();
        m.update(Event::Input(key('c')));
        m.update(Event::Input(key('g')));
        assert_eq!(m.update(Event::Input(key('p'))), vec![Command::ReleaseAll]);
        // Local setpoints cleared: the next presses re-seed from scratch.
        assert_eq!(
            m.update(Event::Input(key('c'))),
            vec![Command::SetCpuW(38.0)]
        );
        assert_eq!(
            m.update(Event::Input(key('g'))),
            vec![Command::SetGpuMaxClock(2985)]
        );
    }

    #[test]
    fn status_syncs_local_setpoints() {
        use crate::control::controller::Mode;
        let mut m = Model::new();
        m.update(Event::Input(key('c'))); // local 38
        let cmds = m.update(Event::Status(ControlStatus {
            mode: Mode::Manual,
            cpu_limit_w: Some(20.0),
            gpu_max_mhz: Some(1500),
            fan_target_rpm: 2500.0,
            flags: vec![],
            calib: None,
            ..ControlStatus::default()
        }));
        assert!(cmds.is_empty());
        // The controller's clamped truth wins: next steps start from it.
        assert_eq!(
            m.update(Event::Input(key('c'))),
            vec![Command::SetCpuW(18.0)]
        );
        assert_eq!(
            m.update(Event::Input(key('g'))),
            vec![Command::SetGpuMaxClock(1395)]
        );
        assert_eq!(m.fan_target_rpm, 2500.0);
    }

    // --- Task 22: calibration keys ---

    /// A ControlStatus in the given mode (rest default).
    fn status_in(mode: Mode) -> ControlStatus {
        ControlStatus {
            mode,
            ..ControlStatus::default()
        }
    }

    #[test]
    fn k_starts_calibration_from_monitor_or_auto() {
        // Default status is Monitor: k emits StartCalibration.
        let mut m = Model::new();
        assert_eq!(
            m.update(Event::Input(key('k'))),
            vec![Command::StartCalibration]
        );

        let mut auto = Model::new();
        auto.update(Event::Status(status_in(Mode::Auto)));
        assert_eq!(
            auto.update(Event::Input(key('k'))),
            vec![Command::StartCalibration]
        );

        // Manual and Calibrating modes: k is ignored.
        for mode in [Mode::Manual, Mode::Calibrating] {
            let mut m = Model::new();
            m.update(Event::Status(status_in(mode)));
            assert_eq!(m.update(Event::Input(key('k'))), vec![], "mode {mode:?}");
        }
    }

    #[test]
    fn manual_and_floor_keys_inert_while_calibrating() {
        let mut m = Model::new();
        m.update(Event::Status(status_in(Mode::Calibrating)));
        // Every manual/floor key: no command AND no local-state drift (the
        // controller would reject the command, so a local step would
        // desynchronize the UI from the applied truth).
        for ev in [
            key('c'),
            shift_key('C'),
            key('g'),
            shift_key('G'),
            key('t'),
            shift_key('T'),
            key('f'),
            shift_key('F'),
            key('d'),
            shift_key('D'),
            key('p'),
        ] {
            assert_eq!(m.update(Event::Input(ev)), vec![], "key {ev:?}");
        }
        assert_eq!(m.fan_target_rpm, DEFAULT_FAN_TARGET_RPM);

        // Calibration over (controller echoes Monitor): the setpoint did
        // not drift, so the next 'c' still re-seeds from 40 W -> 38 W.
        m.update(Event::Status(status_in(Mode::Monitor)));
        assert_eq!(
            m.update(Event::Input(key('c'))),
            vec![Command::SetCpuW(38.0)]
        );
        assert_eq!(m.update(Event::Input(key('f'))), set_floors(14.0, 1000));
    }

    #[test]
    fn esc_aborts_only_while_calibrating() {
        let esc = || Event::Input(KeyEvent::new(KeyCode::Esc, KeyModifiers::NONE));
        // Monitor/Manual: Esc does nothing.
        for mode in [Mode::Monitor, Mode::Manual] {
            let mut m = Model::new();
            m.update(Event::Status(status_in(mode)));
            assert_eq!(m.update(esc()), vec![], "mode {mode:?}");
            assert!(m.running);
        }
        // Calibrating: Esc aborts.
        let mut m = Model::new();
        m.update(Event::Status(status_in(Mode::Calibrating)));
        assert_eq!(m.update(esc()), vec![Command::AbortCalibration]);
        assert!(m.running, "Esc aborts calibration, never quits the app");
    }

    // --- Task 25: auto-mode key ---

    #[test]
    fn a_key_enters_auto_from_monitor_and_manual() {
        // Default (Monitor) status: a emits SetAuto(true).
        let mut m = Model::new();
        assert_eq!(
            m.update(Event::Input(key('a'))),
            vec![Command::SetAuto(true)]
        );

        // Manual mode too: entering Auto replaces the manual limits.
        let mut m = Model::new();
        m.update(Event::Status(status_in(Mode::Manual)));
        assert_eq!(
            m.update(Event::Input(key('a'))),
            vec![Command::SetAuto(true)]
        );
    }

    #[test]
    fn a_key_exits_auto_when_in_auto() {
        let mut m = Model::new();
        m.update(Event::Status(status_in(Mode::Auto)));
        assert_eq!(
            m.update(Event::Input(key('a'))),
            vec![Command::SetAuto(false)]
        );
    }

    #[test]
    fn a_key_blocked_while_calibrating() {
        let mut m = Model::new();
        m.update(Event::Status(status_in(Mode::Calibrating)));
        assert_eq!(m.update(Event::Input(key('a'))), vec![]);
    }

    // --- Task 29: floor keys ---

    fn set_floors(cpu_w: f64, gpu_mhz: u32) -> Vec<Command> {
        vec![Command::SetFloors { cpu_w, gpu_mhz }]
    }

    #[test]
    fn f_keys_step_cpu_floor_and_clamp() {
        let mut m = Model::new();
        // Defaults 15 W / 1000 MHz; both values always travel together.
        assert_eq!(m.update(Event::Input(key('f'))), set_floors(14.0, 1000));
        assert_eq!(
            m.update(Event::Input(shift_key('F'))),
            set_floors(15.0, 1000)
        );
        // Down to the 10 W floor and idempotent at the bound.
        for _ in 0..4 {
            m.update(Event::Input(key('f')));
        }
        assert_eq!(m.update(Event::Input(key('f'))), set_floors(10.0, 1000));
        assert_eq!(m.update(Event::Input(key('f'))), set_floors(10.0, 1000));
        // Up to the 54 W ceiling and idempotent there too.
        for _ in 0..43 {
            m.update(Event::Input(shift_key('F')));
        }
        assert_eq!(
            m.update(Event::Input(shift_key('F'))),
            set_floors(54.0, 1000)
        );
        assert_eq!(
            m.update(Event::Input(shift_key('F'))),
            set_floors(54.0, 1000)
        );
    }

    #[test]
    fn d_keys_step_gpu_floor_and_clamp() {
        let mut m = Model::new();
        // Default GPU floor is already at the 1000 MHz minimum: stepping
        // down stays clamped there.
        assert_eq!(m.update(Event::Input(key('d'))), set_floors(15.0, 1000));
        assert_eq!(
            m.update(Event::Input(shift_key('D'))),
            set_floors(15.0, 1105)
        );
        // Up to the 3090 MHz ceiling: 1105 + 19*105 = 3100 clamps.
        for _ in 0..18 {
            m.update(Event::Input(shift_key('D')));
        }
        assert_eq!(
            m.update(Event::Input(shift_key('D'))),
            set_floors(15.0, 3090)
        );
        assert_eq!(
            m.update(Event::Input(shift_key('D'))),
            set_floors(15.0, 3090)
        );
    }

    #[test]
    fn status_seeds_floor_setpoints() {
        let mut m = Model::new();
        m.update(Event::Status(ControlStatus {
            cpu_floor_w: 20.0,
            gpu_floor_mhz: 1500,
            ..ControlStatus::default()
        }));
        // The controller's sanitized truth wins: next steps start from it.
        assert_eq!(m.update(Event::Input(key('f'))), set_floors(19.0, 1500));
        assert_eq!(m.update(Event::Input(key('d'))), set_floors(19.0, 1395));
    }

    #[test]
    fn q_emits_no_commands() {
        let mut m = Model::new();
        let cmds = m.update(Event::Input(key('q')));
        assert!(
            cmds.is_empty(),
            "Quit is sent by main's shutdown, not update()"
        );
        assert!(!m.running);
    }
}

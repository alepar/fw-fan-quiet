//! TEA-style UI model: single source of UI state, mutated only in update().

use crate::event::{ControlStatus, Event};
use crate::ring::Ring;
use crate::types::Sample;
use crossterm::event::{KeyCode, KeyEventKind, KeyModifiers};

/// Ring capacity: 5 minutes of history at 1 Hz.
pub const RING_CAP: usize = 300;

/// Display-only fan target (RPM) until the controller lands (Task 14).
const DEFAULT_FAN_TARGET_RPM: f64 = 3000.0;

pub struct Model {
    pub max_fan: Ring,
    pub cpu_w: Ring,
    pub gpu_w: Ring,
    pub cpu_temp: Ring,
    pub gpu_temp: Ring,
    pub gpu_mhz: Ring,
    /// Most recent full sample (status panel).
    pub latest: Option<Sample>,
    /// Latest status from the controller.
    pub status: ControlStatus,
    /// false => main loop exits.
    pub running: bool,
    /// Display-only for now.
    pub fan_target_rpm: f64,
}

impl Model {
    pub fn new() -> Self {
        Self {
            max_fan: Ring::new(RING_CAP),
            cpu_w: Ring::new(RING_CAP),
            gpu_w: Ring::new(RING_CAP),
            cpu_temp: Ring::new(RING_CAP),
            gpu_temp: Ring::new(RING_CAP),
            gpu_mhz: Ring::new(RING_CAP),
            latest: None,
            status: ControlStatus::default(),
            running: true,
            fan_target_rpm: DEFAULT_FAN_TARGET_RPM,
        }
    }

    /// The ONLY place UI state changes (TEA update).
    pub fn update(&mut self, ev: Event) {
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
                self.latest = Some(s);
            }
            Event::Input(key) => {
                // Kitty-protocol terminals also deliver Repeat/Release events;
                // only act on presses.
                if key.kind != KeyEventKind::Press {
                    return;
                }
                match (key.code, key.modifiers) {
                    (KeyCode::Char('q'), KeyModifiers::NONE) => self.running = false,
                    // Belt and suspenders; signal handling comes later.
                    (KeyCode::Char('c'), KeyModifiers::CONTROL) => self.running = false,
                    // TODO(task-15): manual-mode keys (m, arrows, ...).
                    _ => {}
                }
            }
            Event::Status(cs) => self.status = cs,
            // Render cadence is driven by the main loop; nothing to do here.
            Event::Tick => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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
        // ControlStatus is an empty placeholder; just verify the event is
        // handled without panicking and the assignment compiles.
        let mut m = Model::new();
        m.update(Event::Status(ControlStatus::default()));
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

    #[test]
    fn tick_is_noop() {
        let mut m = Model::new();
        m.update(Event::Tick);
        assert!(m.running);
        assert!(m.latest.is_none());
        assert_eq!(m.max_fan.len(), 0);
    }
}

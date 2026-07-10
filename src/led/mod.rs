//! LED matrix output: drives the two Framework 16 LED Matrix modules as live
//! CPU/GPU wattage waterfalls (left = CPU, right = GPU). Each panel plots a
//! rolling history — wattage as a horizontal bar on the short axis, time down
//! the tall axis, newest sample on top. A dedicated thread consumes the 1 Hz
//! sample fan-out; **every failure path here is contained** so LED trouble can
//! never disturb the fan controller.
//!
//! Containment rule (load-bearing): once this thread is subscribed to the
//! sampler's fan-out, it must keep draining its channel until disconnect even
//! if both modules die — the sampler treats a dead receiver as fatal and would
//! shut the whole pipeline down. So module loss drops the module but never
//! stops the loop.

pub mod proto;
pub mod render;

use std::path::Path;

use crossbeam_channel::Receiver;

use crate::config::LedConfig;
use crate::event::Event;
use proto::Matrix;
use render::{History, Orient};

/// Per-pixel level for a lit cell. The user-facing brightness knob is the
/// module's global PWM (set at open); lit pixels stay at full per-pixel value
/// and only the partial boundary column is dimmed (in `render::grid`).
const ON_LEVEL: u8 = 255;

/// One panel's mutable output state: its module (None once dropped on error),
/// rolling history, full-scale watts, physical orientation, and log label.
struct Panel {
    matrix: Option<Matrix>,
    history: History,
    full_scale_w: f64,
    orient: Orient,
    label: &'static str,
}

impl Panel {
    /// Feeds one wattage reading: scrolls history and redraws. History keeps
    /// updating even after the module is dropped, so a re-open (future work)
    /// would resume with intact context; a write error drops the module.
    fn update(&mut self, watts: f64, on: u8) {
        let fraction = if self.full_scale_w > 0.0 {
            watts / self.full_scale_w
        } else {
            0.0
        };
        self.history.push(fraction);
        let Some(matrix) = self.matrix.as_mut() else {
            return;
        };
        if let Err(e) = matrix.draw_grid(&render::grid(&self.history, on, self.orient)) {
            tracing::warn!(
                "LED {}: write failed ({e}); dropping module for this run",
                self.label
            );
            self.matrix = None;
        }
    }
}

/// Opens the modules and, if at least one came up, spawns the output thread.
/// Returns `None` (feature inert) when disabled or when neither module opens —
/// the caller then keeps `rx`'s sender out of the sampler fan-out entirely, so
/// no phantom receiver can make the sampler exit.
pub fn spawn(
    config: LedConfig,
    cpu_max_w: f64,
    gpu_max_w: f64,
    rx: Receiver<Event>,
) -> Option<std::thread::JoinHandle<()>> {
    if !config.enabled {
        tracing::info!("LED display disabled by config");
        return None;
    }
    let cpu = open_side("CPU", &config.cpu_port, &config);
    let gpu = open_side("GPU", &config.gpu_port, &config);
    if cpu.is_none() && gpu.is_none() {
        tracing::warn!("LED display: neither module opened, output disabled");
        return None;
    }

    let cpu = Panel {
        matrix: cpu,
        history: History::new(),
        full_scale_w: cpu_max_w,
        orient: Orient {
            flip_time: config.flip_time,
            flip_watts: config.cpu_flip_watts,
        },
        label: "CPU",
    };
    let gpu = Panel {
        matrix: gpu,
        history: History::new(),
        full_scale_w: gpu_max_w,
        orient: Orient {
            flip_time: config.flip_time,
            flip_watts: config.gpu_flip_watts,
        },
        label: "GPU",
    };

    let handle = std::thread::Builder::new()
        .name("led".into())
        .spawn(move || run(rx, cpu, gpu))
        .expect("failed to spawn led thread");
    Some(handle)
}

/// Opens one side's module, logging success/failure. A failure is non-fatal:
/// that side simply stays dark.
fn open_side(label: &str, path: &str, config: &LedConfig) -> Option<Matrix> {
    match Matrix::open(Path::new(path), config.brightness) {
        Ok(matrix) => {
            tracing::info!("LED {label}: driving {path}");
            Some(matrix)
        }
        Err(e) => {
            tracing::warn!("LED {label}: cannot open {path} ({e}); side stays dark");
            None
        }
    }
}

/// Drains the sample stream until the channel disconnects (shutdown), updating
/// each panel's waterfall. Never breaks early — see the containment rule.
fn run(rx: Receiver<Event>, mut cpu: Panel, mut gpu: Panel) {
    for event in rx.iter() {
        if let Event::Sample(sample) = event {
            cpu.update(sample.cpu_pkg_w, ON_LEVEL);
            gpu.update(sample.gpu_w, ON_LEVEL);
        }
    }
    // Clean shutdown (sampler dropped its sender): blank whatever survived so
    // the panels don't hold the last frame after the daemon exits.
    if let Some(m) = cpu.matrix.as_mut() {
        let _ = m.blank();
    }
    if let Some(m) = gpu.matrix.as_mut() {
        let _ = m.blank();
    }
    tracing::debug!("led thread exiting");
}

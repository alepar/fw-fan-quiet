//! Pure render function: Model -> frame. No state lives here; the main loop
//! calls `terminal.draw(|f| view(&model, f))` each iteration.

use ratatui::Frame;
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::symbols::Marker;
use ratatui::text::{Line, Span};
use ratatui::widgets::{Axis, Block, Chart, Dataset, Gauge, GraphType, Paragraph};

use crate::control::controller::{CalibProgressLite, Mode, StatusFlag};
use crate::model::{Model, RING_CAP};
use crate::ring::Ring;

/// Fixed Y bounds per chart: auto-scaling makes live charts jumpy, and these
/// cover the hardware's full envelope (fans max ~7000 RPM, package power well
/// under 120 W, temps below the 110 C trip point, GPU boost under 3.2 GHz).
const FAN_BOUNDS: [f64; 2] = [1000.0, 6000.0];
// Watts chart is percent-of-max like the clocks chart: 100% = the config
// operating maxes carried on the status (`cpu_max_w`/`gpu_max_w`), so both
// series span the full height and the scale matches what control uses.
const TEMP_BOUNDS: [f64; 2] = [20.0, 90.0];
// The clocks chart normalizes each series to percent of that device's max
// clock so both use the full chart height ("dual-scale" on one axis: 100% =
// 5.1 GHz for the CPU, 3.09 GHz for the GPU; absolute MHz live in the title).
const CPU_MAX_CLOCK_MHZ: f64 = 5100.0;
const GPU_MAX_CLOCK_MHZ: f64 = 3090.0;
const PCT_BOUNDS: [f64; 2] = [0.0, 100.0];

pub fn view(model: &Model, frame: &mut Frame) {
    let [header, charts, footer] = Layout::vertical([
        Constraint::Length(1),
        Constraint::Fill(1),
        Constraint::Length(1),
    ])
    .areas(frame.area());

    frame.render_widget(
        Paragraph::new(header_line(model)).style(Style::default().fg(Color::White)),
        header,
    );

    let [top, bottom] = Layout::vertical([Constraint::Fill(1); 2]).areas(charts);
    let [fans_area, watts_area] = Layout::horizontal([Constraint::Fill(1); 2]).areas(top);
    let [temps_area, clock_area] = Layout::horizontal([Constraint::Fill(1); 2]).areas(bottom);

    render_fans(model, frame, fans_area);
    render_watts(model, frame, watts_area);
    render_temps(model, frame, temps_area);
    // The calibration wizard borrows the bottom-right slot (the GPU clock is
    // the least interesting chart mid-calibration); other charts stay live.
    match &model.status.calib {
        Some(progress) => render_calib_wizard(progress, frame, clock_area),
        None => render_clock(model, frame, clock_area),
    }

    let keybar = if model.status.calib.is_some() {
        " q quit  Esc abort calibration"
    } else {
        " q quit  a auto  c/C cpu\u{2213}2W  g/G gpu\u{2213}105MHz  t/T fan\u{2213}250  \
         f/F d/D floors  p release  k calibrate"
    };
    frame.render_widget(
        Paragraph::new(keybar).style(Style::default().fg(Color::DarkGray)),
        footer,
    );
}

/// App name, mode, fan target, commanded limits, active flags and validity
/// warnings for the latest sample. Flags carry their own (loud) styling.
fn header_line(model: &Model) -> Line<'static> {
    let cpu = match model.status.cpu_limit_w {
        Some(w) => format!("cpu\u{2264}{w:.0}W"),
        None => "cpu \u{2013}".into(),
    };
    let gpu = match model.status.gpu_max_mhz {
        Some(mhz) => format!("gpu\u{2264}{mhz}MHz"),
        None => "gpu \u{2013}".into(),
    };
    // Auto is the mode the whole app exists for: style it loud so a glance
    // tells whether the closed loop is driving.
    let mode_span = match model.status.mode {
        Mode::Auto => Span::styled(
            "auto",
            Style::default()
                .fg(Color::Green)
                .add_modifier(Modifier::BOLD),
        ),
        mode => Span::raw(mode.as_str()),
    };
    let mut spans = vec![
        Span::raw(" bazerame-fans | "),
        mode_span,
        Span::raw(format!(
            " | fan target {:.0} rpm | {cpu} | {gpu}",
            model.fan_target_rpm
        )),
    ];
    // The Kalman trim/gain readout is removed: `ControlStatus` no longer
    // carries `trim_rpm`/`gain` (Task 4's type surface). The new fields it
    // gains in their place (`t_star_c`, `budget_w`, ...) are Task 15's
    // header design, not this task's.
    // Severity-first render order: the single-line header has no wrap
    // (ratatui clips at the right edge), so an emergency tripping AFTER
    // milder flags must never be pushed out of view by them. The status
    // Vec itself keeps insertion order (telemetry/tests rely on it); only
    // the spans are sorted.
    let mut flags: Vec<StatusFlag> = model.status.flags.clone();
    flags.sort_by_key(|f| render_priority(*f));
    // With several flags competing for one row, drop the parenthetical
    // "(press ...)" hints so every flag NAME stays visible; a lone flag
    // keeps its full hint.
    let with_hint = flags.len() <= 1;
    for flag in flags {
        spans.push(Span::raw(" | "));
        spans.push(flag_span(flag, with_hint));
    }
    if let Some(s) = &model.latest {
        let mut warnings = Vec::new();
        if !s.fan_valid {
            warnings.push("FAN?");
        }
        if !s.cpu_temp_valid {
            warnings.push("TEMP?");
        }
        if !s.gpu_w_valid {
            warnings.push("GPU?");
        }
        if !warnings.is_empty() {
            spans.push(Span::styled(
                format!(" | sensors lost: {}", warnings.join(" ")),
                Style::default().fg(Color::Red),
            ));
        }
    }
    // Floors: safety config, informational — dim like the trim, and LAST so
    // it can never push a loud flag past a narrow terminal's right edge.
    spans.push(Span::styled(
        format!(
            " | floors {:.0}W/{}MHz",
            model.status.cpu_floor_w, model.status.gpu_floor_mhz
        ),
        Style::default().fg(Color::DarkGray),
    ));
    Line::from(spans)
}

/// Header render priority: lower sorts (and therefore renders) first, so
/// the loudest flag is the one guaranteed to survive right-edge clipping.
/// A strict per-flag order (unlike `controller::Severity`'s three coarse
/// tiers, which the type surface's tests classify by but which alone can't
/// break a tie between two Critical flags): full styling/ordering for the
/// Task 4 flags is Task 15's header design, so they sort after every
/// existing flag for now.
fn render_priority(flag: StatusFlag) -> u8 {
    match flag {
        StatusFlag::ThermalEmergency => 0,
        StatusFlag::SensorLost => 1,
        StatusFlag::TargetUnreachable => 2,
        StatusFlag::LimitNotSticking => 3,
        StatusFlag::NotCalibrated => 4,
        StatusFlag::Resumed => 5,
        StatusFlag::CurveInvalid => 6,
        StatusFlag::EcMismatch => 7,
        StatusFlag::FanctrlLost => 8,
        StatusFlag::GpuHot => 9,
        StatusFlag::NvmeHot => 10,
        StatusFlag::ReadbackBlind => 11,
        StatusFlag::SteepCurve => 12,
    }
}

/// One header span per flag. `with_hint` selects the long form with the
/// parenthetical key hint (single-flag header) or the bare name (several
/// flags competing for one unwrapped row).
fn flag_span(flag: StatusFlag, with_hint: bool) -> Span<'static> {
    let red_bold = Style::default().fg(Color::Red).add_modifier(Modifier::BOLD);
    match flag {
        StatusFlag::LimitNotSticking => Span::styled("LIMIT-SLIP!", red_bold),
        StatusFlag::Resumed => Span::styled("resumed", Style::default().fg(Color::Yellow)),
        StatusFlag::NotCalibrated => Span::styled(
            if with_hint {
                "NOT CALIBRATED (press k to calibrate)"
            } else {
                "NOT CALIBRATED"
            },
            Style::default().fg(Color::Yellow),
        ),
        // No parenthetical hint even alone: with the trim indicator also
        // shown the header would overflow a 120-col terminal ("check intake/
        // ambient" lives in the flag's doc + design notes).
        StatusFlag::TargetUnreachable => Span::styled("TARGET UNREACHABLE", red_bold),
        // Watchdog emergencies: everything was released toward stock and
        // stays released until the user acknowledges (first actuating
        // press re-arms without executing; the second acts normally).
        StatusFlag::ThermalEmergency => Span::styled(
            if with_hint {
                "THERMAL EMERGENCY (press a, c/g or k to acknowledge)"
            } else {
                "THERMAL EMERGENCY"
            },
            red_bold,
        ),
        StatusFlag::SensorLost => Span::styled(
            if with_hint {
                "SENSOR LOST (press a, c/g or k to acknowledge)"
            } else {
                "SENSOR LOST"
            },
            red_bold,
        ),
        // Plain placeholder spans for the Task 4 type surface's new flags:
        // exhaustiveness only (styling/wording is Task 15's header design).
        StatusFlag::FanctrlLost => Span::raw("FANCTRL LOST"),
        StatusFlag::EcMismatch => Span::raw("EC MISMATCH"),
        StatusFlag::SteepCurve => Span::raw("STEEP CURVE"),
        StatusFlag::CurveInvalid => Span::raw("CURVE INVALID"),
        StatusFlag::GpuHot => Span::raw("GPU HOT"),
        StatusFlag::NvmeHot => Span::raw("NVME HOT"),
        StatusFlag::ReadbackBlind => Span::raw("READBACK BLIND"),
    }
}

/// Ring -> chart points split into contiguous valid runs, X = sample index.
/// NaN (invalid reading) ends the current run: rendering each run as its own
/// dataset keeps outages as visible gaps (a `GraphType::Line` dataset would
/// otherwise draw a bridge between the points flanking the NaN run), and the
/// index still advances so the gap has real width.
fn segments(ring: &Ring) -> Vec<Vec<(f64, f64)>> {
    let mut runs = Vec::new();
    let mut current: Vec<(f64, f64)> = Vec::new();
    for (i, v) in ring.iter().enumerate() {
        if v.is_nan() {
            if !current.is_empty() {
                runs.push(std::mem::take(&mut current));
            }
        } else {
            current.push((i as f64, *v));
        }
    }
    if !current.is_empty() {
        runs.push(current);
    }
    runs
}

fn line_dataset(color: Color, data: &[(f64, f64)]) -> Dataset<'_> {
    Dataset::default()
        .marker(Marker::Braille)
        .graph_type(GraphType::Line)
        .style(Style::default().fg(color))
        .data(data)
}

/// One Dataset per contiguous valid segment, so sensor outages render as
/// real gaps (a single dataset would draw a line bridging the missing run).
/// Only the first segment carries the legend name.
fn series<'a>(name: &'a str, color: Color, segments: &'a [Vec<(f64, f64)>]) -> Vec<Dataset<'a>> {
    segments
        .iter()
        .enumerate()
        .map(|(i, seg)| {
            let ds = line_dataset(color, seg);
            if i == 0 { ds.name(name) } else { ds }
        })
        .collect()
}

fn render_chart(
    frame: &mut Frame,
    area: Rect,
    title: String,
    datasets: Vec<Dataset>,
    y_bounds: [f64; 2],
) {
    let y_mid = ((y_bounds[0] + y_bounds[1]) / 2.0).round();
    let chart = Chart::new(datasets)
        .block(Block::bordered().title(title))
        .x_axis(
            Axis::default()
                .bounds([0.0, RING_CAP as f64])
                .labels([
                    "0".to_string(),
                    format!("{}", RING_CAP / 2),
                    format!("{RING_CAP}"),
                ])
                .style(Style::default().fg(Color::DarkGray)),
        )
        .y_axis(
            Axis::default()
                .bounds(y_bounds)
                .labels([
                    format!("{:.0}", y_bounds[0]),
                    format!("{y_mid:.0}"),
                    format!("{:.0}", y_bounds[1]),
                ])
                .style(Style::default().fg(Color::DarkGray)),
        );
    frame.render_widget(chart, area);
}

/// Two-point horizontal guide line at `y` spanning the full X range (fan
/// target and commanded-limit overlays).
fn hline(y: f64) -> [(f64, f64); 2] {
    [(0.0, y), (RING_CAP as f64, y)]
}

/// Base bounds, auto-extended (never shrunk) so every observed value fits:
/// data outside the base range widens the axis instead of clipping.
fn bounds_fit<'a>(
    base: [f64; 2],
    segs: impl IntoIterator<Item = &'a Vec<(f64, f64)>>,
    extra: impl IntoIterator<Item = f64>,
) -> [f64; 2] {
    let mut lo = base[0];
    let mut hi = base[1];
    for v in segs
        .into_iter()
        .flat_map(|run| run.iter().map(|&(_, y)| y))
        .chain(extra)
    {
        if v.is_finite() {
            lo = lo.min(v);
            hi = hi.max(v);
        }
    }
    [lo, hi]
}

fn render_fans(model: &Model, frame: &mut Frame, area: Rect) {
    let fan_segs = segments(&model.max_fan);
    let target_pts = hline(model.fan_target_rpm);
    let bounds = bounds_fit(FAN_BOUNDS, &fan_segs, [model.fan_target_rpm]);
    let title = match &model.latest {
        Some(s) => format!("fans {:.0}/{:.0} rpm", s.fan1_rpm, s.fan2_rpm),
        None => "fans (rpm)".into(),
    };
    let mut datasets = vec![line_dataset(Color::DarkGray, &target_pts).name("target")];
    datasets.extend(series("max fan", Color::Cyan, &fan_segs));
    render_chart(frame, area, title, datasets, bounds);
}

fn render_watts(model: &Model, frame: &mut Frame, area: Rect) {
    let cpu_max_w = model.status.cpu_max_w;
    let gpu_max_w = model.status.gpu_max_w;
    let cpu_segs = to_percent(&segments(&model.cpu_w), cpu_max_w);
    let gpu_segs = to_percent(&segments(&model.gpu_w), gpu_max_w);
    // Commanded CPU limit overlay, on the CPU's percent scale.
    let limit_pts = model
        .status
        .cpu_limit_w
        .map(|w| hline(w / cpu_max_w * 100.0));
    let title = match &model.latest {
        Some(s) => format!(
            "watts cpu {:.1} gpu {:.1} W (% of max)",
            s.cpu_pkg_w, s.gpu_w
        ),
        None => "watts (% of max)".into(),
    };
    let mut datasets = Vec::new();
    if let Some(pts) = &limit_pts {
        datasets.push(line_dataset(Color::DarkGray, pts).name("cpu limit"));
    }
    datasets.extend(series("cpu", Color::Yellow, &cpu_segs));
    datasets.extend(series("gpu", Color::Green, &gpu_segs));
    render_chart(frame, area, title, datasets, PCT_BOUNDS);
}

fn render_temps(model: &Model, frame: &mut Frame, area: Rect) {
    let cpu_segs = segments(&model.cpu_temp);
    let gpu_segs = segments(&model.gpu_temp);
    let title = match &model.latest {
        Some(s) => format!(
            "temps | cpu {:.1}\u{b0}C gpu {:.1}\u{b0}C",
            s.cpu_temp_c, s.gpu_temp_c
        ),
        None => "temps".into(),
    };
    let bounds = bounds_fit(TEMP_BOUNDS, cpu_segs.iter().chain(gpu_segs.iter()), []);
    let mut datasets = series("cpu", Color::Yellow, &cpu_segs);
    datasets.extend(series("gpu", Color::Green, &gpu_segs));
    render_chart(frame, area, title, datasets, bounds);
}

/// Calibration wizard panel: phase, step gauge, load prompt, note, abort
/// hint. Rendered instead of the GPU clock chart while calibrating.
fn render_calib_wizard(progress: &CalibProgressLite, frame: &mut Frame, area: Rect) {
    let block = Block::bordered().title(format!("calibration \u{2014} {}", progress.phase));
    let inner = block.inner(area);
    frame.render_widget(block, area);
    let [gauge_area, load_area, note_area, hint_area] = Layout::vertical([
        Constraint::Length(1),
        Constraint::Length(1),
        Constraint::Length(1),
        Constraint::Length(1),
    ])
    .areas(inner);

    let ratio = if progress.total == 0 {
        0.0
    } else {
        (progress.step as f64 / progress.total as f64).clamp(0.0, 1.0)
    };
    frame.render_widget(
        Gauge::default()
            .gauge_style(Style::default().fg(Color::Cyan).bg(Color::DarkGray))
            .ratio(ratio)
            .label(format!("step {}/{}", progress.step, progress.total)),
        gauge_area,
    );
    if progress.needs_load {
        frame.render_widget(
            Paragraph::new("\u{25b6} START A GPU-HEAVY LOAD (game/benchmark)").style(
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD),
            ),
            load_area,
        );
    }
    frame.render_widget(Paragraph::new(progress.note.clone()), note_area);
    frame.render_widget(
        Paragraph::new("Esc abort").style(Style::default().fg(Color::DarkGray)),
        hint_area,
    );
}

/// Scale every point of pre-split segments to percent of `max` (NaN gaps are
/// already gone by this stage, so plain division is safe).
fn to_percent(segs: &[Vec<(f64, f64)>], max: f64) -> Vec<Vec<(f64, f64)>> {
    segs.iter()
        .map(|run| run.iter().map(|&(x, y)| (x, y / max * 100.0)).collect())
        .collect()
}

fn render_clock(model: &Model, frame: &mut Frame, area: Rect) {
    let gpu_segs = to_percent(&segments(&model.gpu_mhz), GPU_MAX_CLOCK_MHZ);
    let cpu_segs = to_percent(&segments(&model.cpu_mhz), CPU_MAX_CLOCK_MHZ);
    // Commanded GPU max-clock overlay, on the GPU's percent scale.
    let limit_pts = model
        .status
        .gpu_max_mhz
        .map(|mhz| hline(f64::from(mhz) / GPU_MAX_CLOCK_MHZ * 100.0));
    let title = match &model.latest {
        Some(s) => format!(
            "clocks cpu {:.0} gpu {:.0} MHz (% of max)",
            s.cpu_avg_mhz, s.gpu_sm_mhz
        ),
        None => "clocks (% of max)".into(),
    };
    let mut datasets = Vec::new();
    if let Some(pts) = &limit_pts {
        datasets.push(line_dataset(Color::DarkGray, pts).name("gpu max"));
    }
    datasets.extend(series("cpu", Color::Yellow, &cpu_segs));
    datasets.extend(series("gpu", Color::Green, &gpu_segs));
    render_chart(frame, area, title, datasets, PCT_BOUNDS);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::event::Event;
    use crate::types::Sample;
    use ratatui::Terminal;
    use ratatui::backend::TestBackend;

    fn draw(model: &Model) -> Terminal<TestBackend> {
        let mut terminal = Terminal::new(TestBackend::new(120, 40)).unwrap();
        terminal.draw(|f| view(model, f)).unwrap();
        terminal
    }

    fn valid_sample(v: f64) -> Sample {
        Sample {
            fan1_rpm: v,
            fan2_rpm: v * 0.9,
            cpu_pkg_w: v * 0.01,
            gpu_w: v * 0.02,
            cpu_temp_c: v * 0.02,
            gpu_temp_c: v * 0.015,
            gpu_sm_mhz: v,
            fan_valid: true,
            cpu_temp_valid: true,
            gpu_w_valid: true,
            gpu_temp_valid: true,
            gpu_mhz_valid: true,
            ..Default::default()
        }
    }

    fn row_text(terminal: &Terminal<TestBackend>, y: u16) -> String {
        let buf = terminal.backend().buffer();
        (0..buf.area.width)
            .map(|x| buf.cell((x, y)).unwrap().symbol())
            .collect()
    }

    #[test]
    fn view_renders_without_panic_on_empty_and_full_model() {
        draw(&Model::new());
        let mut m = Model::new();
        for i in 0..400 {
            m.update(Event::Sample(valid_sample(f64::from(i))));
        }
        draw(&m);
    }

    #[test]
    fn invalid_fan_sample_puts_warning_in_header() {
        let mut m = Model::new();
        m.update(Event::Sample(Sample {
            cpu_temp_valid: true,
            gpu_w_valid: true,
            ..Default::default() // fan_valid: false
        }));
        let terminal = draw(&m);
        let header = row_text(&terminal, 0);
        assert!(header.contains("FAN?"), "header was: {header:?}");
        assert!(!header.contains("TEMP?"), "header was: {header:?}");
    }

    #[test]
    fn all_sensors_valid_means_no_warnings() {
        let mut m = Model::new();
        m.update(Event::Sample(valid_sample(1500.0)));
        let terminal = draw(&m);
        let header = row_text(&terminal, 0);
        assert!(!header.contains('?'), "header was: {header:?}");
    }

    #[test]
    fn bounds_fit_extends_only_when_data_exceeds() {
        // Inside base range: unchanged.
        let inside = vec![vec![(0.0, 2000.0), (1.0, 5500.0)]];
        assert_eq!(bounds_fit([1000.0, 6000.0], &inside, []), [1000.0, 6000.0]);
        // Above: hi extends; below: lo extends; extras (target line) count too.
        let above = vec![vec![(0.0, 6800.0)]];
        assert_eq!(bounds_fit([1000.0, 6000.0], &above, []), [1000.0, 6800.0]);
        let below = vec![vec![(0.0, 15.0)]];
        assert_eq!(bounds_fit([20.0, 90.0], &below, []), [15.0, 90.0]);
        assert_eq!(
            bounds_fit([1000.0, 6000.0], &[], [6500.0]),
            [1000.0, 6500.0]
        );
    }

    #[test]
    fn segments_split_on_nan_runs() {
        let mut ring = Ring::new(8);
        for v in [1.0, f64::NAN, f64::NAN, 2.0, 3.0] {
            ring.push(v);
        }
        assert_eq!(
            segments(&ring),
            vec![vec![(0.0, 1.0)], vec![(3.0, 2.0), (4.0, 3.0)]],
            "a NaN run must split the series so the chart shows a gap, not a bridge"
        );
    }

    #[test]
    fn segments_of_all_valid_ring_is_one_run() {
        let mut ring = Ring::new(4);
        ring.push(1.0);
        ring.push(2.0);
        assert_eq!(segments(&ring), vec![vec![(0.0, 1.0), (1.0, 2.0)]]);
    }

    #[test]
    fn segments_of_empty_or_all_nan_ring_is_empty() {
        assert!(segments(&Ring::new(4)).is_empty());
        let mut ring = Ring::new(4);
        ring.push(f64::NAN);
        assert!(segments(&ring).is_empty());
    }

    #[test]
    fn header_shows_mode_limits_and_flags() {
        use crate::control::ControlStatus;
        use crate::control::controller::{Mode, StatusFlag};
        let mut m = Model::new();
        m.update(Event::Status(ControlStatus {
            mode: Mode::Manual,
            cpu_limit_w: Some(20.0),
            gpu_max_mhz: Some(1500),
            fan_target_rpm: 2500.0,
            flags: vec![StatusFlag::LimitNotSticking, StatusFlag::Resumed],
            calib: None,
            ..ControlStatus::default()
        }));
        let terminal = draw(&m);
        let header = row_text(&terminal, 0);
        assert!(header.contains("manual"), "header was: {header:?}");
        assert!(header.contains("cpu\u{2264}20W"), "header was: {header:?}");
        assert!(
            header.contains("gpu\u{2264}1500MHz"),
            "header was: {header:?}"
        );
        assert!(header.contains("LIMIT-SLIP!"), "header was: {header:?}");
        assert!(header.contains("resumed"), "header was: {header:?}");
        assert!(header.contains("2500"), "header was: {header:?}");
    }

    #[test]
    fn header_shows_monitor_mode_and_dashes_without_limits() {
        let terminal = draw(&Model::new());
        let header = row_text(&terminal, 0);
        assert!(header.contains("monitor"), "header was: {header:?}");
        assert!(header.contains("cpu \u{2013}"), "header was: {header:?}");
        assert!(header.contains("gpu \u{2013}"), "header was: {header:?}");
        assert!(!header.contains("LIMIT-SLIP!"), "header was: {header:?}");
    }

    #[test]
    fn limit_slip_flag_is_red_bold() {
        use crate::control::ControlStatus;
        use crate::control::controller::{Mode, StatusFlag};
        use ratatui::style::Modifier;
        let mut m = Model::new();
        m.update(Event::Status(ControlStatus {
            mode: Mode::Manual,
            cpu_limit_w: Some(20.0),
            gpu_max_mhz: None,
            fan_target_rpm: 3000.0,
            flags: vec![StatusFlag::LimitNotSticking],
            calib: None,
            ..ControlStatus::default()
        }));
        let terminal = draw(&m);
        let header = row_text(&terminal, 0);
        let x = header.find("LIMIT-SLIP!").expect("flag text present") as u16;
        let cell = terminal.backend().buffer().cell((x, 0)).unwrap();
        assert_eq!(cell.fg, Color::Red);
        assert!(cell.modifier.contains(Modifier::BOLD));
    }

    #[test]
    fn thermal_emergency_flag_is_red_bold_with_acknowledge_hint() {
        use crate::control::ControlStatus;
        use crate::control::controller::{Mode, StatusFlag};
        use ratatui::style::Modifier;
        let mut m = Model::new();
        m.update(Event::Status(ControlStatus {
            mode: Mode::Monitor,
            cpu_limit_w: None,
            gpu_max_mhz: None,
            fan_target_rpm: 3000.0,
            flags: vec![StatusFlag::ThermalEmergency],
            calib: None,
            ..ControlStatus::default()
        }));
        let terminal = draw(&m);
        let header = row_text(&terminal, 0);
        let x = header
            .find("THERMAL EMERGENCY (press a, c/g or k to acknowledge)")
            .expect("flag text present") as u16;
        let cell = terminal.backend().buffer().cell((x, 0)).unwrap();
        assert_eq!(cell.fg, Color::Red);
        assert!(cell.modifier.contains(Modifier::BOLD));
    }

    #[test]
    fn sensor_lost_flag_is_red_bold_with_acknowledge_hint() {
        use crate::control::ControlStatus;
        use crate::control::controller::{Mode, StatusFlag};
        use ratatui::style::Modifier;
        let mut m = Model::new();
        m.update(Event::Status(ControlStatus {
            mode: Mode::Monitor,
            cpu_limit_w: None,
            gpu_max_mhz: None,
            fan_target_rpm: 3000.0,
            flags: vec![StatusFlag::SensorLost],
            calib: None,
            ..ControlStatus::default()
        }));
        let terminal = draw(&m);
        let header = row_text(&terminal, 0);
        let x = header
            .find("SENSOR LOST (press a, c/g or k to acknowledge)")
            .expect("flag text present") as u16;
        let cell = terminal.backend().buffer().cell((x, 0)).unwrap();
        assert_eq!(cell.fg, Color::Red);
        assert!(cell.modifier.contains(Modifier::BOLD));
    }

    #[test]
    fn emergency_stays_visible_at_120_cols_with_many_flags() {
        use crate::control::ControlStatus;
        use crate::control::controller::{Mode, StatusFlag};
        // Worst case from the review: THERMAL EMERGENCY trips LAST, after
        // Auto + four other flags already fill the header. Without
        // severity-first ordering (and hint dropping) the emergency text
        // starts past column 120 and ratatui clips it invisible.
        let mut m = Model::new();
        m.update(Event::Status(ControlStatus {
            mode: Mode::Auto,
            cpu_limit_w: Some(17.0),
            gpu_max_mhz: Some(1653),
            fan_target_rpm: 3000.0,
            flags: vec![
                StatusFlag::Resumed,
                StatusFlag::LimitNotSticking,
                StatusFlag::TargetUnreachable,
                StatusFlag::NotCalibrated,
                StatusFlag::ThermalEmergency,
            ],
            calib: None,
            ..ControlStatus::default()
        }));
        let terminal = draw(&m); // 120x40 TestBackend
        let header = row_text(&terminal, 0);
        let emergency = header
            .find("THERMAL EMERGENCY")
            .expect("emergency must survive clipping") as u16;
        // Rendered FIRST among the flags despite being last in the Vec.
        for other in ["TARGET UNREACHABLE", "LIMIT-SLIP!", "resumed"] {
            if let Some(x) = header.find(other) {
                assert!(
                    emergency < x as u16,
                    "{other} must render after the emergency: {header:?}"
                );
            }
        }
        // More than one flag active: parenthetical hints are dropped.
        assert!(!header.contains("(press"), "header was: {header:?}");
        // Loud styling still applies.
        let cell = terminal.backend().buffer().cell((emergency, 0)).unwrap();
        assert_eq!(cell.fg, Color::Red);
        assert!(cell.modifier.contains(Modifier::BOLD));
    }

    #[test]
    fn keybar_lists_manual_key_hints() {
        let terminal = draw(&Model::new());
        let footer = row_text(&terminal, 39);
        for hint in [
            "q quit",
            "a auto",
            "c/C cpu\u{2213}2W",
            "g/G gpu\u{2213}105MHz",
            "t/T fan\u{2213}250",
            "f/F d/D floors",
            "p release",
            "k calibrate",
        ] {
            assert!(footer.contains(hint), "footer was: {footer:?}");
        }
        assert!(!footer.contains("Esc abort"), "footer was: {footer:?}");
    }

    // --- Task 29: floors in the header ---

    #[test]
    fn header_shows_floors_dim() {
        use crate::control::ControlStatus;
        // Defaults render right away (floors are always shown).
        let terminal = draw(&Model::new());
        let header = row_text(&terminal, 0);
        assert!(
            header.contains("floors 15W/1000MHz"),
            "header was: {header:?}"
        );
        let x = header.find("floors 15W/1000MHz").unwrap() as u16;
        let cell = terminal.backend().buffer().cell((x, 0)).unwrap();
        assert_eq!(cell.fg, Color::DarkGray, "floors must render dim");

        // Edited floors show through the echoed status.
        let mut m = Model::new();
        m.update(Event::Status(ControlStatus {
            cpu_floor_w: 20.0,
            gpu_floor_mhz: 1210,
            ..ControlStatus::default()
        }));
        let header = row_text(&draw(&m), 0);
        assert!(
            header.contains("floors 20W/1210MHz"),
            "header was: {header:?}"
        );
    }

    // --- Task 25: auto mode in the header ---

    #[test]
    fn auto_mode_renders_green_bold() {
        use crate::control::ControlStatus;
        use crate::control::controller::Mode;
        let mut m = Model::new();
        m.update(Event::Status(ControlStatus {
            mode: Mode::Auto,
            cpu_limit_w: Some(17.0),
            gpu_max_mhz: Some(1653),
            fan_target_rpm: 3000.0,
            flags: vec![],
            calib: None,
            ..ControlStatus::default()
        }));
        let terminal = draw(&m);
        let header = row_text(&terminal, 0);
        let x = header.find("auto").expect("mode text present") as u16;
        let cell = terminal.backend().buffer().cell((x, 0)).unwrap();
        assert_eq!(cell.fg, Color::Green);
        assert!(cell.modifier.contains(Modifier::BOLD));
        // Allocation shows through the existing limit fields.
        assert!(header.contains("cpu\u{2264}17W"), "header was: {header:?}");
        assert!(
            header.contains("gpu\u{2264}1653MHz"),
            "header was: {header:?}"
        );
    }

    // --- Task 26: TargetUnreachable in the header ---
    // (the trim/gain readout these tests used to also exercise is removed —
    // Task 4 drops `trim_rpm`/`gain` from `ControlStatus`.)

    #[test]
    fn target_unreachable_flag_is_red_bold() {
        use crate::control::ControlStatus;
        use crate::control::controller::{Mode, StatusFlag};
        use ratatui::style::Modifier;
        let mut m = Model::new();
        m.update(Event::Status(ControlStatus {
            mode: Mode::Auto,
            cpu_limit_w: Some(15.0),
            gpu_max_mhz: Some(1200),
            fan_target_rpm: 3000.0,
            flags: vec![StatusFlag::TargetUnreachable],
            calib: None,
            ..ControlStatus::default()
        }));
        let terminal = draw(&m);
        let header = row_text(&terminal, 0);
        let x = header
            .find("TARGET UNREACHABLE")
            .expect("flag text present") as u16;
        let cell = terminal.backend().buffer().cell((x, 0)).unwrap();
        assert_eq!(cell.fg, Color::Red);
        assert!(cell.modifier.contains(Modifier::BOLD));
    }

    // Task 27's ModelDistrust flag is removed from the Task 4 type surface
    // (`model_distrust_flag_is_red_bold` tested it here; StatusFlag no
    // longer has that variant, so there is nothing left to render).

    #[test]
    fn not_calibrated_flag_renders_yellow_hint() {
        use crate::control::ControlStatus;
        use crate::control::controller::{Mode, StatusFlag};
        let mut m = Model::new();
        m.update(Event::Status(ControlStatus {
            mode: Mode::Monitor,
            cpu_limit_w: None,
            gpu_max_mhz: None,
            fan_target_rpm: 3000.0,
            flags: vec![StatusFlag::NotCalibrated],
            calib: None,
            ..ControlStatus::default()
        }));
        let terminal = draw(&m);
        let header = row_text(&terminal, 0);
        let x = header.find("NOT CALIBRATED").expect("hint present") as u16;
        assert!(
            header.contains("press k to calibrate"),
            "header was: {header:?}"
        );
        let cell = terminal.backend().buffer().cell((x, 0)).unwrap();
        assert_eq!(cell.fg, Color::Yellow);
    }

    // --- Task 22: calibration wizard panel ---

    /// All buffer rows joined with newlines (wizard text spans several rows).
    fn all_text(terminal: &Terminal<TestBackend>) -> String {
        (0..40)
            .map(|y| row_text(terminal, y))
            .collect::<Vec<_>>()
            .join("\n")
    }

    fn calibrating_model(needs_load: bool) -> Model {
        use crate::control::ControlStatus;
        use crate::control::controller::{CalibProgressLite, Mode};
        let mut m = Model::new();
        // Live data in the rings: the other three charts keep rendering.
        for i in 0..50 {
            m.update(Event::Sample(valid_sample(f64::from(i) * 30.0)));
        }
        m.update(Event::Status(ControlStatus {
            mode: Mode::Calibrating,
            cpu_limit_w: Some(30.0),
            gpu_max_mhz: Some(1950),
            fan_target_rpm: 3000.0,
            flags: vec![],
            calib: Some(CalibProgressLite {
                phase: "matrix".into(),
                step: 4,
                total: 11,
                needs_load,
                note: "matrix point 5/11: cpu 5 W, gpu 35 W".into(),
            }),
            ..ControlStatus::default()
        }));
        m
    }

    #[test]
    fn wizard_panel_renders_with_needs_load_prompt() {
        let terminal = draw(&calibrating_model(true));
        let text = all_text(&terminal);
        assert!(text.contains("calibration \u{2014} matrix"), "text: {text}");
        assert!(text.contains("step 4/11"), "text: {text}");
        assert!(
            text.contains("START A GPU-HEAVY LOAD"),
            "needs_load prompt missing: {text}"
        );
        assert!(text.contains("matrix point 5/11"), "text: {text}");
        assert!(text.contains("Esc abort"), "text: {text}");
        // Header reflects the mode; keybar switched to the abort hint.
        assert!(row_text(&terminal, 0).contains("calibrating"));
        assert!(row_text(&terminal, 39).contains("Esc abort"));
        assert!(!row_text(&terminal, 39).contains("k calibrate"));
    }

    #[test]
    fn wizard_panel_without_needs_load_hides_prompt() {
        let terminal = draw(&calibrating_model(false));
        let text = all_text(&terminal);
        assert!(text.contains("calibration \u{2014} matrix"), "text: {text}");
        assert!(
            !text.contains("START A GPU-HEAVY LOAD"),
            "prompt must be hidden: {text}"
        );
    }

    #[test]
    fn wizard_panel_survives_tiny_areas_and_zero_total() {
        use crate::control::ControlStatus;
        use crate::control::controller::{CalibProgressLite, Mode};
        let mut m = Model::new();
        m.update(Event::Status(ControlStatus {
            mode: Mode::Calibrating,
            cpu_limit_w: None,
            gpu_max_mhz: None,
            fan_target_rpm: 3000.0,
            flags: vec![],
            calib: Some(CalibProgressLite {
                phase: "aborted".into(),
                step: 0,
                total: 0,
                needs_load: true,
                note: String::new(),
            }),
            ..ControlStatus::default()
        }));
        // Tiny terminal: every wizard sub-area degenerates; must not panic.
        let mut terminal = Terminal::new(TestBackend::new(20, 6)).unwrap();
        terminal.draw(|f| view(&m, f)).unwrap();
        draw(&m); // and the normal size with total == 0 (gauge ratio 0)
    }

    #[test]
    fn commanded_limit_lines_render_without_panic() {
        use crate::control::ControlStatus;
        use crate::control::controller::Mode;
        // Limits set with EMPTY rings (Status before first Sample) must not panic.
        let mut m = Model::new();
        m.update(Event::Status(ControlStatus {
            mode: Mode::Manual,
            cpu_limit_w: Some(20.0),
            gpu_max_mhz: Some(1500),
            fan_target_rpm: 3000.0,
            flags: vec![],
            calib: None,
            ..ControlStatus::default()
        }));
        draw(&m);
        // Some limits + data in the rings...
        let mut m = Model::new();
        for i in 0..50 {
            m.update(Event::Sample(valid_sample(f64::from(i) * 30.0)));
        }
        m.update(Event::Status(ControlStatus {
            mode: Mode::Manual,
            cpu_limit_w: Some(54.0),
            gpu_max_mhz: Some(3090),
            fan_target_rpm: 3000.0,
            flags: vec![],
            calib: None,
            ..ControlStatus::default()
        }));
        draw(&m);
        // ...and back to None mid-session (release).
        m.update(Event::Status(ControlStatus::default()));
        draw(&m);
    }

    #[test]
    fn nan_in_rings_renders_without_panic() {
        let mut m = Model::new();
        for i in 0..50 {
            let mut s = valid_sample(f64::from(i) * 30.0);
            if i % 5 == 0 {
                // NaN lands in the fan/temp/gpu_w rings for these samples.
                s.fan_valid = false;
                s.cpu_temp_valid = false;
                s.gpu_w_valid = false;
            }
            m.update(Event::Sample(s));
        }
        assert!(
            m.max_fan.iter().any(|v| v.is_nan()),
            "test setup: NaN must be present"
        );
        draw(&m);
    }
}

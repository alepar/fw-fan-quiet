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
const FAN_BOUNDS: [f64; 2] = [0.0, 7000.0];
const WATT_BOUNDS: [f64; 2] = [0.0, 120.0];
const TEMP_BOUNDS: [f64; 2] = [0.0, 110.0];
const MHZ_BOUNDS: [f64; 2] = [0.0, 3200.0];

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
         p release  k calibrate"
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
    // Ambient trim (Auto mode): informational, so dim — the loud version of
    // this signal is the TargetUnreachable flag below.
    if model.status.trim_rpm != 0.0 {
        spans.push(Span::styled(
            format!(" | trim {:+.0}rpm", model.status.trim_rpm),
            Style::default().fg(Color::DarkGray),
        ));
    }
    for flag in &model.status.flags {
        spans.push(Span::raw(" | "));
        spans.push(match flag {
            StatusFlag::LimitNotSticking => Span::styled(
                "LIMIT-SLIP!",
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
            StatusFlag::Resumed => Span::styled("resumed", Style::default().fg(Color::Yellow)),
            StatusFlag::NotCalibrated => Span::styled(
                "NOT CALIBRATED (press k to calibrate)",
                Style::default().fg(Color::Yellow),
            ),
            // No parenthetical hint: with the trim indicator also shown the
            // header would overflow a 120-col terminal ("check intake/
            // ambient" lives in the flag's doc + design notes).
            StatusFlag::TargetUnreachable => Span::styled(
                "TARGET UNREACHABLE",
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
            // The model's predictions have been persistently wrong for 5+
            // minutes: RLS frozen, trim at half gain (recalibrate if this
            // persists — the hint lives in the flag's doc, not the header).
            StatusFlag::ModelDistrust => Span::styled(
                "MODEL DISTRUST",
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
        });
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
    Line::from(spans)
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

fn render_fans(model: &Model, frame: &mut Frame, area: Rect) {
    let fan_segs = segments(&model.max_fan);
    let target_pts = hline(model.fan_target_rpm);
    let title = match &model.latest {
        Some(s) => format!("fans {:.0}/{:.0} rpm", s.fan1_rpm, s.fan2_rpm),
        None => "fans (rpm)".into(),
    };
    let mut datasets = vec![line_dataset(Color::DarkGray, &target_pts).name("target")];
    datasets.extend(series("max fan", Color::Cyan, &fan_segs));
    render_chart(frame, area, title, datasets, FAN_BOUNDS);
}

fn render_watts(model: &Model, frame: &mut Frame, area: Rect) {
    let cpu_segs = segments(&model.cpu_w);
    let gpu_segs = segments(&model.gpu_w);
    // Commanded CPU limit overlay (same pattern as the fan target line).
    let limit_pts = model.status.cpu_limit_w.map(hline);
    let title = match &model.latest {
        Some(s) => format!("watts | cpu {:.1} W gpu {:.1} W", s.cpu_pkg_w, s.gpu_w),
        None => "watts".into(),
    };
    let mut datasets = Vec::new();
    if let Some(pts) = &limit_pts {
        datasets.push(line_dataset(Color::DarkGray, pts).name("cpu limit"));
    }
    datasets.extend(series("cpu", Color::Yellow, &cpu_segs));
    datasets.extend(series("gpu", Color::Green, &gpu_segs));
    render_chart(frame, area, title, datasets, WATT_BOUNDS);
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
    let mut datasets = series("cpu", Color::Red, &cpu_segs);
    datasets.extend(series("gpu", Color::Magenta, &gpu_segs));
    render_chart(frame, area, title, datasets, TEMP_BOUNDS);
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

fn render_clock(model: &Model, frame: &mut Frame, area: Rect) {
    let segs = segments(&model.gpu_mhz);
    // Commanded GPU max-clock overlay.
    let limit_pts = model.status.gpu_max_mhz.map(|mhz| hline(f64::from(mhz)));
    let title = match &model.latest {
        Some(s) => format!("gpu clock {:.0} MHz", s.gpu_sm_mhz),
        None => "gpu clock (MHz)".into(),
    };
    let mut datasets = Vec::new();
    if let Some(pts) = &limit_pts {
        datasets.push(line_dataset(Color::DarkGray, pts).name("max"));
    }
    datasets.extend(series("sm", Color::Blue, &segs));
    render_chart(frame, area, title, datasets, MHZ_BOUNDS);
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
            trim_rpm: 0.0,
            flags: vec![StatusFlag::LimitNotSticking, StatusFlag::Resumed],
            calib: None,
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
            trim_rpm: 0.0,
            flags: vec![StatusFlag::LimitNotSticking],
            calib: None,
        }));
        let terminal = draw(&m);
        let header = row_text(&terminal, 0);
        let x = header.find("LIMIT-SLIP!").expect("flag text present") as u16;
        let cell = terminal.backend().buffer().cell((x, 0)).unwrap();
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
            "p release",
            "k calibrate",
        ] {
            assert!(footer.contains(hint), "footer was: {footer:?}");
        }
        assert!(!footer.contains("Esc abort"), "footer was: {footer:?}");
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
            trim_rpm: 0.0,
            flags: vec![],
            calib: None,
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

    // --- Task 26: trim + TargetUnreachable in the header ---

    #[test]
    fn header_shows_trim_dim_when_nonzero() {
        use crate::control::ControlStatus;
        use crate::control::controller::Mode;
        let mut m = Model::new();
        m.update(Event::Status(ControlStatus {
            mode: Mode::Auto,
            cpu_limit_w: Some(17.0),
            gpu_max_mhz: Some(1653),
            fan_target_rpm: 3000.0,
            trim_rpm: 123.0,
            flags: vec![],
            calib: None,
        }));
        let terminal = draw(&m);
        let header = row_text(&terminal, 0);
        assert!(header.contains("trim +123rpm"), "header was: {header:?}");
        let x = header.find("trim +123rpm").unwrap() as u16;
        let cell = terminal.backend().buffer().cell((x, 0)).unwrap();
        assert_eq!(cell.fg, Color::DarkGray, "trim must render dim");

        // Negative trim shows its sign too.
        m.update(Event::Status(ControlStatus {
            mode: Mode::Auto,
            cpu_limit_w: Some(17.0),
            gpu_max_mhz: Some(1653),
            fan_target_rpm: 3000.0,
            trim_rpm: -17.0,
            flags: vec![],
            calib: None,
        }));
        let header = row_text(&draw(&m), 0);
        assert!(header.contains("trim -17rpm"), "header was: {header:?}");
    }

    #[test]
    fn header_hides_trim_when_zero() {
        // Default status has trim 0: no trim clutter in the header.
        let header = row_text(&draw(&Model::new()), 0);
        assert!(!header.contains("trim"), "header was: {header:?}");
    }

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
            trim_rpm: 400.0,
            flags: vec![StatusFlag::TargetUnreachable],
            calib: None,
        }));
        let terminal = draw(&m);
        let header = row_text(&terminal, 0);
        let x = header
            .find("TARGET UNREACHABLE")
            .expect("flag text present") as u16;
        // The saturated trim shows alongside (the flag means "pinned at max").
        assert!(header.contains("trim +400rpm"), "header was: {header:?}");
        let cell = terminal.backend().buffer().cell((x, 0)).unwrap();
        assert_eq!(cell.fg, Color::Red);
        assert!(cell.modifier.contains(Modifier::BOLD));
    }

    // --- Task 27: ModelDistrust in the header ---

    #[test]
    fn model_distrust_flag_is_red_bold() {
        use crate::control::ControlStatus;
        use crate::control::controller::{Mode, StatusFlag};
        use ratatui::style::Modifier;
        let mut m = Model::new();
        m.update(Event::Status(ControlStatus {
            mode: Mode::Auto,
            cpu_limit_w: Some(17.0),
            gpu_max_mhz: Some(1653),
            fan_target_rpm: 3000.0,
            trim_rpm: 200.0,
            flags: vec![StatusFlag::ModelDistrust],
            calib: None,
        }));
        let terminal = draw(&m);
        let header = row_text(&terminal, 0);
        let x = header.find("MODEL DISTRUST").expect("flag text present") as u16;
        let cell = terminal.backend().buffer().cell((x, 0)).unwrap();
        assert_eq!(cell.fg, Color::Red);
        assert!(cell.modifier.contains(Modifier::BOLD));
    }

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
            trim_rpm: 0.0,
            flags: vec![StatusFlag::NotCalibrated],
            calib: None,
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
            trim_rpm: 0.0,
            flags: vec![],
            calib: Some(CalibProgressLite {
                phase: "matrix".into(),
                step: 4,
                total: 11,
                needs_load,
                note: "matrix point 5/11: cpu 5 W, gpu 35 W".into(),
            }),
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
            trim_rpm: 0.0,
            flags: vec![],
            calib: Some(CalibProgressLite {
                phase: "aborted".into(),
                step: 0,
                total: 0,
                needs_load: true,
                note: String::new(),
            }),
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
            trim_rpm: 0.0,
            flags: vec![],
            calib: None,
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
            trim_rpm: 0.0,
            flags: vec![],
            calib: None,
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

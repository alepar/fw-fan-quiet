//! Pure render function: Model -> frame. No state lives here; the main loop
//! calls `terminal.draw(|f| view(&model, f))` each iteration.

use ratatui::Frame;
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Style};
use ratatui::symbols::Marker;
use ratatui::widgets::{Axis, Block, Chart, Dataset, GraphType, Paragraph};

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
    render_clock(model, frame, clock_area);

    frame.render_widget(
        Paragraph::new(" q quit").style(Style::default().fg(Color::DarkGray)),
        footer,
    );
}

/// App name, fan target, and validity warnings for the latest sample.
fn header_line(model: &Model) -> String {
    let mut line = format!(
        " bazerame-fans | fan target {:.0} rpm",
        model.fan_target_rpm
    );
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
            line.push_str(" | sensors lost: ");
            line.push_str(&warnings.join(" "));
        }
    }
    line
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

fn render_fans(model: &Model, frame: &mut Frame, area: Rect) {
    let fan_segs = segments(&model.max_fan);
    let target_pts = [
        (0.0, model.fan_target_rpm),
        (RING_CAP as f64, model.fan_target_rpm),
    ];
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
    let title = match &model.latest {
        Some(s) => format!("watts | cpu {:.1} W gpu {:.1} W", s.cpu_pkg_w, s.gpu_w),
        None => "watts".into(),
    };
    let mut datasets = series("cpu", Color::Yellow, &cpu_segs);
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

fn render_clock(model: &Model, frame: &mut Frame, area: Rect) {
    let segs = segments(&model.gpu_mhz);
    let title = match &model.latest {
        Some(s) => format!("gpu clock {:.0} MHz", s.gpu_sm_mhz),
        None => "gpu clock (MHz)".into(),
    };
    render_chart(
        frame,
        area,
        title,
        series("sm", Color::Blue, &segs),
        MHZ_BOUNDS,
    );
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

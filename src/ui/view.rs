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
use crate::types::{
    GainsSource, TelemetryBound, TelemetryDeviceName, TelemetryFlag,
    TelemetryTStarState,
};

/// Fixed Y bounds per chart: auto-scaling makes live charts jumpy, and these
/// cover the hardware's full envelope (fans max ~7000 RPM, package power well
/// under 120 W, temps below the 110 C trip point, GPU boost under 3.2 GHz).
const FAN_BOUNDS: [f64; 2] = [1000.0, 6000.0];
// Watts charts use stable presentation scales so live graphs do not jump.
const TEMP_BOUNDS: [f64; 2] = [20.0, 90.0];
// The clocks chart normalizes each series to percent of that device's max
// clock so both use the full chart height ("dual-scale" on one axis: 100% =
// 5.1 GHz for the CPU, 3.09 GHz for the GPU; absolute MHz live in the title).
const CPU_MAX_CLOCK_MHZ: f64 = 5100.0;
const GPU_MAX_CLOCK_MHZ: f64 = 3090.0;
const PCT_BOUNDS: [f64; 2] = [0.0, 100.0];

fn main_areas(area: Rect) -> [Rect; 3] {
    Layout::vertical([
        Constraint::Length(1),
        Constraint::Fill(1),
        Constraint::Length(1),
    ])
    .areas(area)
}

pub fn calib_result_scroll_max(model: &Model, area: Rect) -> u16 {
    if model.status.calib.is_some() { return 0; }
    let Some(outcome) = &model.status.calib_outcome else { return 0; };
    let [_, charts, _] = main_areas(area);
    let [_, bottom] = Layout::vertical([Constraint::Fill(1); 2]).areas(charts);
    outcome_scroll_max(outcome, bottom)
}

pub fn view(model: &Model, frame: &mut Frame) {
    let [header, charts, footer] = main_areas(frame.area());

    frame.render_widget(
        Paragraph::new(header_line(model)).style(Style::default().fg(Color::White)),
        header,
    );

    let [top, bottom] = Layout::vertical([Constraint::Fill(1); 2]).areas(charts);
    let [fans_area, watts_area] = Layout::horizontal([Constraint::Fill(1); 2]).areas(top);
    let [temps_area, clock_area] = Layout::horizontal([Constraint::Fill(1); 2]).areas(bottom);

    render_fans(model, frame, fans_area);
    render_watts(model, frame, watts_area);
    if model.status.calib.is_none() && let Some(outcome) = &model.status.calib_outcome {
        render_calib_outcome(outcome, model.calib_result_scroll, frame, bottom);
    } else {
        render_temps(model, frame, temps_area);
        // The calibration wizard borrows the bottom-right slot (the GPU clock is
        // the least interesting chart mid-calibration); other charts stay live.
        match &model.status.calib {
            Some(progress) => render_calib_wizard(progress, frame, clock_area),
            None => render_clock(model, frame, clock_area),
        }
    }

    let keybar = if model.status.calib.is_some() {
        " q quit  Esc abort calibration"
    } else if model.status.calib_outcome.is_some() {
        " q quit  Esc dismiss result  Up/Down scroll  Home top  a auto  k calibrate"
    } else {
        " q quit  a auto  c/C cpu\u{2213}2W  g/G gpu\u{2213}105MHz  t/T fan\u{2213}250  \
         f/F d/D floors  p release  k calibrate"
    };
    frame.render_widget(
        Paragraph::new(keybar).style(Style::default().fg(Color::DarkGray)),
        footer,
    );
}

/// Mode, fan target, commanded limits, active flags and validity
/// warnings for the latest sample. Flags carry their own (loud) styling.
fn header_line(model: &Model) -> Line<'static> {
    let cpu = match model.status.cpu_limit_w {
        Some(w) => format!("{w:.1}/{:.0}W", model.status.cpu_max_w),
        None => format!("—/{:.0}W", model.status.cpu_max_w),
    };
    let gpu = match model.status.gpu_max_mhz {
        Some(mhz) => format!("{:.1}/{:.1}GHz", f64::from(mhz) / 1000.0, f64::from(model.status.gpu_ceiling_mhz) / 1000.0),
        None => format!("—/{:.1}GHz", f64::from(model.status.gpu_ceiling_mhz) / 1000.0),
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
        Span::raw(" "),
        mode_span,
    ];
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
    // Diagnostic flags stay ahead of optional control details on narrow screens.
    for flag in &model.status.telemetry_flags {
        spans.push(Span::raw(format!(" | {}", flag_text(flag))));
    }
    if model.status.t_star_c.is_some() || model.status.tstar_state.is_some() {
        spans.push(Span::raw(format!(
            " | T* {} {}",
            opt_temp(model.status.t_star_c),
            model.status.tstar_state.map(tstar_state_name).unwrap_or("—"),
        )));
    }
    for (name, device, cap) in [
        ("CPU", model.status.cpu.as_ref(), cpu),
        ("GPU", model.status.gpu.as_ref(), gpu),
    ] {
        spans.push(Span::raw(format!(" | {name} ")));
        spans.push(error_span(device.and_then(|d| d.err_c)));
        spans.push(Span::raw(format!(" - {cap}")));
        if device.is_some_and(|d| d.gains_source != GainsSource::Fitted) {
            spans.push(Span::styled(" unfitted", Style::default().fg(Color::Red)));
        }
    }
    let current_rpm = model.latest.as_ref().filter(|s| s.fan_valid)
        .map(|s| s.max_fan_rpm()).filter(|rpm| rpm.is_finite());
    let fan_color = match current_rpm {
        Some(rpm) if rpm < model.fan_target_rpm => Color::Green,
        Some(_) => Color::Yellow,
        None => Color::DarkGray,
    };
    spans.push(Span::raw(" | fan "));
    spans.push(Span::styled(
        current_rpm.map(|rpm| format!("{rpm:.0}")).unwrap_or_else(|| "—".into()),
        Style::default().fg(fan_color),
    ));
    spans.push(Span::raw(format!("/{:.0} rpm", model.fan_target_rpm)));
    let ambient = model.latest.as_ref().filter(|s| s.ec_valid)
        .and_then(|s| s.ec.as_ref())
        .and_then(|ec| ec.all.iter().find(|(label, _)| label.as_str() == "ambient_f75303@4d"))
        .map(|(_, temperature)| *temperature);
    let nvme = model.latest.as_ref().and_then(|s| s.nvme_temp_c);
    spans.push(Span::raw(" | ambient "));
    spans.push(temperature_span(ambient, model.status.t_star_c));
    spans.push(Span::raw(" | NVMe "));
    spans.push(temperature_span(nvme, Some(model.nvme_max_c)));
    Line::from(spans)
}

/// Header render priority: lower sorts (and therefore renders) first, so
/// the loudest flag is the one guaranteed to survive right-edge clipping.
/// A strict per-flag order (unlike `controller::Severity`'s three coarse
/// tiers, which classify but can't alone break a tie between two flags of
/// the same tier): Critical, then Warning, then Info, existing flags
/// keeping their established relative order within a tier and the six new
/// Task 15 flags slotted in by the severities design §3.5 states for them
/// (`CURVE INVALID`'s is stated by the acceptance criteria directly:
/// warning, outranking the `STEEP CURVE` info flag). Agrees with
/// `controller::flag_severity` on every flag, `NvmeHot` included: design
/// §3.5's header text is explicit that `NVME HOT` renders as a warning
/// alongside `GPU HOT`. (For one epic the two diverged — this file carried
/// the warning while `flag_severity` still said Info, because the flag's
/// task could not touch `controller.rs`; reconciled after the epic landed.)
fn render_priority(flag: StatusFlag) -> u8 {
    match flag {
        // Critical
        StatusFlag::ThermalEmergency => 0,
        StatusFlag::SensorLost => 1,
        StatusFlag::TargetUnreachable => 2,
        // Warning
        StatusFlag::LimitNotSticking => 3,
        StatusFlag::NotCalibrated => 4,
        StatusFlag::CurveInvalid => 5,
        StatusFlag::EcMismatch => 6,
        StatusFlag::FanctrlLost => 7,
        StatusFlag::GpuHot => 8,
        StatusFlag::NvmeHot => 9,
        // Info
        StatusFlag::Resumed => 10,
        StatusFlag::SteepCurve => 11,
        StatusFlag::ReadbackBlind => 12,
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
        // The five Task 15 warning-severity flags (design §3.5 + the
        // acceptance criteria's explicit call on CURVE INVALID): yellow,
        // one step down from the red-bold Critical flags above, matching
        // NOT CALIBRATED's existing warning-tier look.
        StatusFlag::CurveInvalid => {
            Span::styled("CURVE INVALID", Style::default().fg(Color::Yellow))
        }
        StatusFlag::EcMismatch => Span::styled("EC MISMATCH", Style::default().fg(Color::Yellow)),
        StatusFlag::FanctrlLost => Span::styled("FANCTRL LOST", Style::default().fg(Color::Yellow)),
        StatusFlag::GpuHot => Span::styled("GPU HOT", Style::default().fg(Color::Yellow)),
        StatusFlag::NvmeHot => Span::styled("NVME HOT", Style::default().fg(Color::Yellow)),
        // The two Task 15 info-severity flags (`READBACK BLIND` per the
        // brief, `STEEP CURVE` per design §3.5): a plain, unbolded gray —
        // visible but clearly a notch below the warnings above.
        StatusFlag::SteepCurve => Span::styled("STEEP CURVE", Style::default().fg(Color::Gray)),
        StatusFlag::ReadbackBlind => {
            Span::styled("READBACK BLIND", Style::default().fg(Color::Gray))
        }
    }
}

#[cfg(test)]
pub(crate) fn test_status_flag_text(flag: StatusFlag) -> String {
    flag_span(flag, false).content.into_owned()
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
    target: Option<(f64, Color)>,
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
    // The bordered chart reserves two inner rows for x-axis/labels. Match
    // its Braille canvas's four-dot vertical resolution before mapping to a
    // terminal row, so the tick aligns with the actual plotted target.
    if area.width >= 8 && area.height >= 5
        && let Some((target, color)) = target.filter(|(v, _)| v.is_finite())
        && y_bounds[1] > y_bounds[0]
        && (y_bounds[0]..=y_bounds[1]).contains(&target)
    {
        let plot_height = area.height - 4;
        let dot = ((y_bounds[1] - target) / (y_bounds[1] - y_bounds[0])
            * (f64::from(plot_height) * 4.0 - 1.0)).round() as u16;
        let y = area.y + 1 + dot / 4;
        frame.render_widget(
            Paragraph::new("<").style(Style::default().fg(color).add_modifier(Modifier::BOLD)),
            Rect::new(area.right() - 1, y, 1, 1),
        );
    }
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
    let target_segs = segments(&model.fan_target);
    let bounds = bounds_fit(FAN_BOUNDS, fan_segs.iter().chain(target_segs.iter()), [model.fan_target_rpm]);
    let title = match &model.latest {
        Some(s) => format!("fans {:.0}/{:.0} rpm", s.fan1_rpm, s.fan2_rpm),
        None => "fans (rpm)".into(),
    };
    let mut datasets = series("target", Color::DarkGray, &target_segs);
    datasets.extend(series("max fan", Color::Cyan, &fan_segs));
    render_chart(frame, area, title, datasets, bounds, Some((model.fan_target_rpm, Color::Cyan)));
}

/// Error is target minus group temperature: arrows show desired direction,
/// never a measured trend. Neutral only means zero at displayed precision.
fn error_span(error: Option<f64>) -> Span<'static> {
    match error.filter(|v| v.is_finite()) {
        Some(v) if v >= 0.05 => Span::styled(format!("↑{v:+.1}°C"), Style::default().fg(Color::Cyan)),
        Some(v) if v <= -0.05 => Span::styled(format!("↓{v:+.1}°C"), Style::default().fg(Color::Yellow)),
        Some(_) => Span::styled("≈0.0°C", Style::default().fg(Color::Green)),
        None => Span::styled("—", Style::default().fg(Color::DarkGray)),
    }
}

fn temperature_span(value: Option<f64>, threshold: Option<f64>) -> Span<'static> {
    let value = value.filter(|v| v.is_finite());
    let color = match (value, threshold.filter(|t| t.is_finite())) {
        (Some(v), Some(t)) if v < t => Color::Green,
        (Some(_), Some(_)) => Color::Yellow,
        _ => Color::DarkGray,
    };
    Span::styled(opt_temp(value), Style::default().fg(color))
}

fn opt_temp(value: Option<f64>) -> String {
    value.map(|v| format!("{v:.1}\u{b0}C")).unwrap_or_else(|| "—".into())
}

// Exhaustive names make a new wire variant a compile error until it gains a
// readable rendering.
fn tstar_state_name(value: TelemetryTStarState) -> &'static str {
    match value { TelemetryTStarState::Curve => "Curve", TelemetryTStarState::Held => "Held", TelemetryTStarState::Uncontrollable => "Uncontrollable", TelemetryTStarState::Released => "Released" }
}
fn bound_name(value: TelemetryBound) -> &'static str {
    match value { TelemetryBound::Floor => "floor", TelemetryBound::Max => "max" }
}
fn device_name(value: TelemetryDeviceName) -> &'static str {
    match value { TelemetryDeviceName::Cpu => "CPU", TelemetryDeviceName::Gpu => "GPU" }
}
fn polarity(value: bool) -> &'static str { if value { "active" } else { "clear" } }
fn flag_text(value: &TelemetryFlag) -> String { match value {
    TelemetryFlag::ArgmaxUncontrollable { label, active } => format!("argmax uncontrollable {label} ({})", polarity(*active)),
    TelemetryFlag::ArgmaxStuck { label, active } => format!("argmax stuck {label} ({})", polarity(*active)),
    TelemetryFlag::EcUnknownLabel { label, active } => format!("EC unknown label {label} ({})", polarity(*active)),
    TelemetryFlag::EcImplausible { label, active } => format!("EC implausible {label} ({})", polarity(*active)),
    TelemetryFlag::EcUncontrollableUnavailable { active } => format!("EC uncontrollable unavailable ({})", polarity(*active)),
    TelemetryFlag::GroupLost { device, active } => format!("{} group lost ({})", device_name(*device), polarity(*active)),
    TelemetryFlag::DeviceUnreachable { device, bound, active } => format!("{} unreachable at {} ({})", device_name(*device), bound_name(*bound), polarity(*active)),
    TelemetryFlag::TargetUnreachable { bound, active } => format!("target unreachable at {} ({})", bound_name(*bound), polarity(*active)),
    TelemetryFlag::SteepCurve { active } => format!("steep curve ({})", polarity(*active)),
    TelemetryFlag::Legacy { flag, active } => format!("{flag} ({})", polarity(*active)),
} }

/// Measured CPU/GPU power history remains useful alongside independent caps;
/// the prior shared-control display was removed.
fn render_watts(model: &Model, frame: &mut Frame, area: Rect) {
    let cpu_max_w = model.status.cpu_max_w;
    let gpu_scale_w = crate::config::GPU_POWER_SCALE_W;
    let cpu_segs = to_percent(&segments(&model.cpu_w), cpu_max_w);
    let gpu_segs = to_percent(&segments(&model.gpu_w), gpu_scale_w);
    let cap_segs = to_percent(&segments(&model.cpu_cap), cpu_max_w);
    let title = match &model.latest {
        Some(s) => format!(
            "watts cpu {:.1} gpu {:.1} W (% of display scale)",
            s.cpu_pkg_w, s.gpu_w
        ),
        None => "watts (% of display scale)".into(),
    };
    let mut datasets = series("cpu cap", Color::DarkGray, &cap_segs);
    datasets.extend(series("cpu", Color::Yellow, &cpu_segs));
    datasets.extend(series("gpu", Color::Green, &gpu_segs));
    render_chart(frame, area, title, datasets, PCT_BOUNDS, model.status.cpu_limit_w.map(|w| (w / cpu_max_w * 100.0, Color::Yellow)));
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
    let target_segs = segments(&model.temp_target);
    let bounds = bounds_fit(TEMP_BOUNDS, cpu_segs.iter().chain(gpu_segs.iter()).chain(target_segs.iter()), model.status.t_star_c);
    let mut datasets = series("T*", Color::DarkGray, &target_segs);
    datasets.extend(series("cpu", Color::Yellow, &cpu_segs));
    datasets.extend(series("gpu", Color::Green, &gpu_segs));
    render_chart(frame, area, title, datasets, bounds, model.status.t_star_c.map(|t| (t, Color::White)));
}

/// Calibration wizard panel: phase, step gauge, load prompt, note, abort
/// hint. Rendered instead of the GPU clock chart while calibrating.
fn render_calib_wizard(progress: &CalibProgressLite, frame: &mut Frame, area: Rect) {
    let block = Block::bordered().title(format!("calibration \u{2014} {}", progress.phase));
    let inner = block.inner(area);
    frame.render_widget(block, area);
    let [gauge_area, load_area, note_area, hint_area] = Layout::vertical([
        Constraint::Length(1),
        Constraint::Length(2),
        Constraint::Min(1),
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
    frame.render_widget(
        Paragraph::new("Keep GPU load >90% during calibration\nSuggested GPU load: gpu_burn").wrap(ratatui::widgets::Wrap { trim: false }).style(
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD),
        ),
        load_area,
    );
    frame.render_widget(Paragraph::new(progress.note.clone()).wrap(ratatui::widgets::Wrap { trim: false }), note_area);
    frame.render_widget(
        Paragraph::new("Esc abort").style(Style::default().fg(Color::DarkGray)),
        hint_area,
    );
}

fn outcome_scroll_max(outcome: &crate::calib::runner::CalibOutcome, area: Rect) -> u16 {
    let inner = Block::bordered().inner(area);
    let lines = Paragraph::new(outcome.details()).wrap(ratatui::widgets::Wrap { trim: false })
        .line_count(inner.width);
    lines.saturating_sub(usize::from(inner.height)).min(usize::from(u16::MAX)) as u16
}

fn render_calib_outcome(outcome: &crate::calib::runner::CalibOutcome, scroll: u16, frame: &mut Frame, area: Rect) {
    let color = match outcome.title() {
        "success" => Color::Green,
        "partial success" => Color::Yellow,
        _ => Color::Red,
    };
    let block = Block::bordered().title(format!("calibration — {}", outcome.title()))
        .border_style(Style::default().fg(color));
    let paragraph = Paragraph::new(outcome.details())
        .wrap(ratatui::widgets::Wrap { trim: false });
    let scroll = scroll.min(outcome_scroll_max(outcome, area));
    frame.render_widget(paragraph.scroll((scroll, 0)).block(block), area);
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
    let cap_segs = to_percent(&segments(&model.gpu_cap), GPU_MAX_CLOCK_MHZ);
    let title = match &model.latest {
        Some(s) => format!(
            "clocks cpu {:.0} gpu {:.0} MHz (% of max)",
            s.cpu_avg_mhz, s.gpu_sm_mhz
        ),
        None => "clocks (% of max)".into(),
    };
    let mut datasets = series("gpu max", Color::DarkGray, &cap_segs);
    datasets.extend(series("cpu", Color::Yellow, &cpu_segs));
    datasets.extend(series("gpu", Color::Green, &gpu_segs));
    render_chart(frame, area, title, datasets, PCT_BOUNDS, model.status.gpu_max_mhz.map(|mhz| (f64::from(mhz) / GPU_MAX_CLOCK_MHZ * 100.0, Color::Green)));
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::event::Event;
    use crate::types::{Sample, TelemetryDevice, TelemetryHold, TelemetrySelected};
    use ratatui::Terminal;
    use ratatui::backend::TestBackend;

    /// Default test width, 200 cols. Widened from 120 (Task 15): the new
    /// prior shared-control segment always renders ~50 more
    /// characters into the single-line header, on top of what already
    /// filled a 120-col terminal in prior tasks.
    /// `emergency_stays_visible_when_flags_would_overflow_the_header` below
    /// keeps its own dedicated, narrower terminal — the one test that is
    /// deliberately about clipping.
    fn draw(model: &Model) -> Terminal<TestBackend> {
        draw_size(model, 200, 40)
    }

    fn draw_size(model: &Model, width: u16, height: u16) -> Terminal<TestBackend> {
        let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
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

    /// Column (cell) index of `needle`'s first match in `text` at or after
    /// column `from`. `row_text` joins each cell's symbol into a `String`
    /// one-cell-per-`char`, so a column is a CHAR position — `str::find`'s
    /// BYTE offset desyncs from it the moment a multi-byte glyph (°, ·, ≤,
    /// –, →, the header is full of them) appears before the match, which
    /// silently mis-locates cell-color assertions instead of failing them.
    fn find_col_from(text: &str, needle: &str, from: usize) -> Option<usize> {
        let chars: Vec<char> = text.chars().collect();
        let needle: Vec<char> = needle.chars().collect();
        if needle.is_empty() || from + needle.len() > chars.len() {
            return None;
        }
        (from..=chars.len() - needle.len()).find(|&i| chars[i..i + needle.len()] == needle[..])
    }

    fn find_col(text: &str, needle: &str) -> Option<usize> {
        find_col_from(text, needle, 0)
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
    fn header_cap_denominators_use_configured_maxima() {
        let mut m = Model::new();
        m.status.cpu_max_w = 50.0;
        m.status.gpu_ceiling_mhz = 3000;
        m.status.cpu_limit_w = Some(44.0);
        m.status.gpu_max_mhz = Some(2800);
        let header = row_text(&draw(&m), 0);
        assert!(header.contains("44.0/50W"));
        assert!(header.contains("2.8/3.0GHz"));
        m.status.cpu_limit_w = None;
        m.status.gpu_max_mhz = None;
        let header = row_text(&draw(&m), 0);
        assert!(header.contains("—/50W"));
        assert!(header.contains("—/3.0GHz"));
    }

    #[test]
    fn header_fan_current_target_and_color_follow_valid_maximum() {
        for (rpm, valid, text, color) in [
            (3123.0, true, "3123", Color::Green),
            (3750.0, true, "3750", Color::Yellow),
            (4000.0, true, "4000", Color::Yellow),
            (4000.0, false, "—", Color::DarkGray),
        ] {
            let mut m = Model::new();
            m.fan_target_rpm = 3750.0;
            let mut sample = valid_sample(1000.0);
            sample.fan2_rpm = rpm;
            sample.fan_valid = valid;
            m.update(Event::Sample(sample));
            let terminal = draw(&m);
            let header = row_text(&terminal, 0);
            let expected = format!("fan {text}/3750 rpm");
            let x = find_col(&header, &expected).expect(&header) as u16 + 4;
            assert_eq!(terminal.backend().buffer().cell((x, 0)).unwrap().fg, color);
            let target_x = x + text.chars().count() as u16 + 1;
            assert_eq!(terminal.backend().buffer().cell((target_x, 0)).unwrap().fg, Color::White);
        }
        assert!(row_text(&draw(&Model::new()), 0).contains("fan —/3000 rpm"));
    }

    #[test]
    fn target_tick_aligns_with_overlapping_trace_and_omits_unknown() {
        for target in [0.0, 25.0, 50.0, 75.0, 100.0] {
            let mut terminal = Terminal::new(TestBackend::new(40, 12)).unwrap();
            let points = [(0.0, target), (RING_CAP as f64, target)];
            terminal.draw(|f| render_chart(f, f.area(), "test".into(),
                vec![line_dataset(Color::Green, &points)], [0.0, 100.0], Some((target, Color::White)))).unwrap();
            let buf = terminal.backend().buffer();
            let tick_y = (1..9).find(|&y| buf.cell((39, y)).unwrap().symbol() == "<").unwrap();
            let trace = buf.cell((38, tick_y)).unwrap();
            assert_eq!(trace.fg, Color::Green);
            assert_ne!(trace.symbol(), " ");
        }
        for target in [None, Some(f64::NAN), Some(f64::INFINITY), Some(101.0)] {
            let mut terminal = Terminal::new(TestBackend::new(40, 12)).unwrap();
            terminal.draw(|f| render_chart(f, f.area(), "test".into(), vec![], [0.0, 100.0], target.map(|t| (t, Color::White)))).unwrap();
            assert!(!all_text(&terminal).contains('<'));
        }
        for (width, height) in [(1, 1), (8, 4), (8, 5), (20, 8)] {
            let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
            terminal.draw(|f| render_chart(f, f.area(), "test".into(), vec![], [0.0, 100.0], Some((50.0, Color::White)))).unwrap();
        }
    }

    #[test]
    fn target_ticks_follow_live_targets_on_all_chart_borders() {
        let mut m = Model::new();
        m.fan_target_rpm = 6000.0;
        m.status.cpu_max_w = 54.0;
        m.status.cpu_limit_w = Some(54.0);
        m.status.t_star_c = Some(90.0);
        m.status.gpu_max_mhz = Some(3090);
        let terminal = draw_size(&m, 200, 40);
        for (x, y, color) in [(99, 2, Color::Cyan), (199, 2, Color::Yellow), (99, 21, Color::White), (199, 21, Color::Green)] {
            let cell = terminal.backend().buffer().cell((x, y)).unwrap();
            assert_eq!(cell.symbol(), "<", "missing tick at {x},{y}");
            assert_eq!(cell.fg, color);
        }
        m.status.cpu_limit_w = None;
        m.status.t_star_c = None;
        m.status.gpu_max_mhz = None;
        let terminal = draw_size(&m, 200, 40);
        assert_eq!(all_text(&terminal).matches('<').count(), 1);
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
        assert!(header.contains("CPU — - 20.0/54W"), "header was: {header:?}");
        assert!(
            header.contains("GPU — - 1.5/3.1GHz"),
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
        assert!(header.contains("CPU — - —"), "header was: {header:?}");
        assert!(header.contains("GPU — - —"), "header was: {header:?}");
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
        let x = find_col(&header, "LIMIT-SLIP!").expect("flag text present") as u16;
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
        let x = find_col(
            &header,
            "THERMAL EMERGENCY (press a, c/g or k to acknowledge)",
        )
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
        let x = find_col(&header, "SENSOR LOST (press a, c/g or k to acknowledge)")
            .expect("flag text present") as u16;
        let cell = terminal.backend().buffer().cell((x, 0)).unwrap();
        assert_eq!(cell.fg, Color::Red);
        assert!(cell.modifier.contains(Modifier::BOLD));
    }

    #[test]
    fn emergency_stays_visible_when_flags_would_overflow_the_header() {
        use crate::control::ControlStatus;
        use crate::control::controller::{Mode, StatusFlag};
        // Worst case from the review: THERMAL EMERGENCY trips LAST, after
        // Auto + four other flags already fill the header. Without
        // severity-first ordering (and hint dropping) the emergency text
        // starts past the terminal's right edge and ratatui clips it
        // invisible. A dedicated (not the shared `draw()`) 170-col
        // terminal, deliberately narrow enough that the base status fields
        // plus all five flags do not fit without severity-first ordering.
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
        let mut terminal = Terminal::new(TestBackend::new(170, 40)).unwrap();
        terminal.draw(|f| view(&m, f)).unwrap();
        let header = row_text(&terminal, 0);
        let emergency =
            find_col(&header, "THERMAL EMERGENCY").expect("emergency must survive clipping") as u16;
        // Rendered FIRST among the flags despite being last in the Vec.
        for other in ["TARGET UNREACHABLE", "LIMIT-SLIP!", "resumed"] {
            if let Some(x) = find_col(&header, other) {
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
    fn temperature_color_respects_threshold_and_unknown_values() {
        for (value, threshold, color) in [
            (Some(76.8), Some(79.0), Color::Green),
            (Some(79.0), Some(79.0), Color::Yellow),
            (Some(81.0), Some(79.0), Color::Yellow),
            (Some(85.0), Some(90.0), Color::Green),
            (Some(90.0), Some(90.0), Color::Yellow),
            (None, Some(90.0), Color::DarkGray),
            (Some(70.0), None, Color::DarkGray),
            (Some(f64::NAN), Some(90.0), Color::DarkGray),
        ] {
            assert_eq!(temperature_span(value, threshold).style.fg, Some(color));
        }
    }

    #[test]
    fn header_omits_name_and_floors() {
        let header = row_text(&draw(&Model::new()), 0);
        assert!(!header.contains("fw-fan-quiet"));
        assert!(!header.contains("floors"));
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
        let x = find_col(&header, "auto").expect("mode text present") as u16;
        let cell = terminal.backend().buffer().cell((x, 0)).unwrap();
        assert_eq!(cell.fg, Color::Green);
        assert!(cell.modifier.contains(Modifier::BOLD));
        // Allocation shows through the existing limit fields.
        assert!(header.contains("CPU — - 17.0/54W"), "header was: {header:?}");
        assert!(
            header.contains("GPU — - 1.7/3.1GHz"),
            "header was: {header:?}"
        );
    }

    // --- Task 26: TargetUnreachable in the header ---
    // (the trim/gain readout these tests used to also exercise is removed —
    // Task 4 drops the old bias/gain fields from `ControlStatus`.)

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
        let x = find_col(&header, "TARGET UNREACHABLE").expect("flag text present") as u16;
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
        let x = find_col(&header, "NOT CALIBRATED").expect("hint present") as u16;
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
        (0..terminal.backend().buffer().area.height)
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
    fn completed_calibration_panel_shows_changes_and_rejections() {
        use crate::calib::runner::{CalibGainChange, CalibOutcome};
        use crate::calib::step::CalibDevice;
        use crate::control::device_loop::Gains;
        let mut model = Model::new();
        model.status.calib_outcome = Some(CalibOutcome {
            changes: vec![CalibGainChange {
                device: CalibDevice::Gpu,
                before: Gains { kc: 22.22, ti_s: 32.56 },
                after: Gains { kc: 24.0, ti_s: 35.0 },
            }],
            errors: vec!["CPU fit rejected: fitted response 2.00C is below 3C".into()],
            applied: true,
            saved: true,
            notes: Vec::new(),
            ..Default::default()
        });
        let text = all_text(&draw(&model));
        for expected in ["partial success", "GPU: Kc 22.2200 -> 24.0000", "CPU: unchanged", "fitted response 2.00C", "Calibration gains updated and saved"] {
            assert!(text.contains(expected), "missing {expected}: {text}");
        }
        assert!(!text.contains("Esc abort"));
        assert!(text.contains("Esc dismiss result"));
        for title in ["success", "failed"] {
            let outcome = model.status.calib_outcome.as_mut().unwrap();
            if title == "success" {
                let mut cpu = outcome.changes[0].clone();
                cpu.device = CalibDevice::Cpu;
                outcome.changes.push(cpu);
                outcome.errors.clear();
            } else {
                outcome.applied = false;
                outcome.errors.push("calibration aborted".into());
            }
            assert!(all_text(&draw_size(&model, 80, 24)).contains(title));
            draw_size(&model, 10, 5);
        }
        let text = all_text(&draw(&calibrating_model(false)));
        assert!(text.contains("gpu_burn"), "{text}");
        let small = all_text(&draw_size(&calibrating_model(false), 80, 24));
        assert!(small.contains("gpu_burn"), "{small}");
    }

    #[test]
    fn long_calibration_rejection_can_be_read_to_the_end() {
        let mut model = Model::new();
        model.status.calib_outcome = Some(crate::calib::runner::CalibOutcome {
            errors: vec![format!("{} FINAL_REJECTION_DETAIL", "timeout gate failed; ".repeat(80))],
            ..Default::default()
        });
        assert!(!all_text(&draw_size(&model, 80, 24)).contains("FINAL_REJECTION_DETAIL"));
        model.calib_result_scroll = u16::MAX;
        assert!(all_text(&draw_size(&model, 80, 24)).contains("FINAL_REJECTION_DETAIL"));
    }

    #[test]
    fn wizard_panel_renders_with_needs_load_prompt() {
        let terminal = draw(&calibrating_model(true));
        let text = all_text(&terminal);
        assert!(text.contains("calibration \u{2014} matrix"), "text: {text}");
        assert!(text.contains("step 4/11"), "text: {text}");
        assert!(
            text.contains("Keep GPU load >90% during calibration"),
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
    fn wizard_panel_keeps_gpu_requirement_visible_without_load_request() {
        let terminal = draw(&calibrating_model(false));
        let text = all_text(&terminal);
        assert!(text.contains("calibration \u{2014} matrix"), "text: {text}");
        assert!(
            text.contains("Keep GPU load >90% during calibration"),
            "GPU requirement must remain visible: {text}"
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

    // --- Task 13: per-device loop panel ---

    fn decision(selected: TelemetrySelected, hold: TelemetryHold, gains_source: GainsSource) -> TelemetryDevice {
        TelemetryDevice { group_c: Some(72.5), err_c: Some(-1.5), thermal: 31.0, shadow: 36.0, cap: 31.0, selected, hold, gains_source }
    }

    fn two_loop_status(state: TelemetryTStarState) -> crate::control::ControlStatus {
        crate::control::ControlStatus {
            t_star_c: Some(71.0), tstar_state: Some(state), cpu_limit_w: Some(31.0), gpu_max_mhz: Some(2100),
            cpu: Some(decision(TelemetrySelected::Thermal, TelemetryHold::None, GainsSource::Config)),
            gpu: Some(decision(TelemetrySelected::Shadow, TelemetryHold::Shadow, GainsSource::Fitted)),
            ..crate::control::ControlStatus::default()
        }
    }

    #[test]
    fn loop_status_fits_header_and_reclaims_panel_space() {
        for state in [TelemetryTStarState::Curve, TelemetryTStarState::Held, TelemetryTStarState::Uncontrollable, TelemetryTStarState::Released] {
            let mut status = two_loop_status(state);
            status.cpu.as_mut().unwrap().err_c = Some(18.6);
            let mut model = Model::new(); model.update(Event::Status(status));
            let terminal = draw_size(&model, 200, 40);
            let header = row_text(&terminal, 0);
            for expected in ["T* 71.0°C", tstar_state_name(state), "CPU ↑+18.6°C - 31.0/54W unfitted", "GPU ↓-1.5°C - 2.1/3.1GHz"] {
                assert!(header.contains(expected), "missing {expected}: {header}");
            }
            for (label, color) in [("↑+18.6°C", Color::Cyan), ("↓-1.5°C", Color::Yellow)] {
                let x = find_col(&header, label).unwrap() as u16;
                assert_eq!(terminal.backend().buffer().cell((x, 0)).unwrap().fg, color);
            }
            assert!(row_text(&terminal, 1).contains("fans"));
            assert!(!all_text(&terminal).contains("two-loop control"));
        }
    }

    #[test]
    fn view_fixtures_cover_each_gains_source_and_structured_flag() {
        for (source, text) in [(GainsSource::Config, " unfitted"), (GainsSource::Fitted, ""), (GainsSource::Default, " unfitted")] {
            let mut status = two_loop_status(TelemetryTStarState::Uncontrollable);
            status.cpu = Some(decision(TelemetrySelected::Thermal, TelemetryHold::None, source));
            let mut model = Model::new(); model.update(Event::Status(status));
            let terminal = draw_size(&model, 200, 40);
            let header = row_text(&terminal, 0);
            assert!(header.contains(&format!("CPU ↓-1.5°C - 31.0/54W{text}")));
            assert!(!header.contains(" fitted"));
            if source != GainsSource::Fitted {
                let x = find_col(&header, "unfitted").unwrap() as u16;
                assert_eq!(terminal.backend().buffer().cell((x, 0)).unwrap().fg, Color::Red);
            } else {
                assert!(!header.contains("unfitted"));
            }
        }
        for flag in [
            TelemetryFlag::ArgmaxUncontrollable { label: "ambient".into(), active: true }, TelemetryFlag::ArgmaxStuck { label: "gpu_vr".into(), active: true }, TelemetryFlag::EcUnknownLabel { label: "mystery".into(), active: true }, TelemetryFlag::EcImplausible { label: "gpu_mem".into(), active: true }, TelemetryFlag::EcUncontrollableUnavailable { active: true }, TelemetryFlag::GroupLost { device: TelemetryDeviceName::Gpu, active: true }, TelemetryFlag::DeviceUnreachable { device: TelemetryDeviceName::Cpu, bound: TelemetryBound::Floor, active: true }, TelemetryFlag::TargetUnreachable { bound: TelemetryBound::Max, active: true }, TelemetryFlag::SteepCurve { active: true }, TelemetryFlag::Legacy { flag: "EC MISMATCH".into(), active: false },
        ] {
            let expected = super::flag_text(&flag);
            let mut status = two_loop_status(TelemetryTStarState::Released); status.telemetry_flags = vec![flag];
            let mut model = Model::new(); model.update(Event::Status(status));
            assert!(all_text(&draw_size(&model, 80, 24)).contains(&expected), "missing {expected}");
        }
    }

    // --- Task 15: the seven new StatusFlags ---

    fn status_with_flag(flag: StatusFlag) -> crate::control::ControlStatus {
        use crate::control::ControlStatus;
        ControlStatus {
            flags: vec![flag],
            ..ControlStatus::default()
        }
    }

    fn flag_text(flag: StatusFlag) -> &'static str {
        match flag {
            StatusFlag::FanctrlLost => "FANCTRL LOST",
            StatusFlag::EcMismatch => "EC MISMATCH",
            StatusFlag::SteepCurve => "STEEP CURVE",
            StatusFlag::CurveInvalid => "CURVE INVALID",
            StatusFlag::GpuHot => "GPU HOT",
            StatusFlag::NvmeHot => "NVME HOT",
            StatusFlag::ReadbackBlind => "READBACK BLIND",
            _ => unreachable!("not one of the seven Task 15 flags"),
        }
    }

    #[test]
    fn each_new_flag_renders_its_name() {
        for flag in [
            StatusFlag::FanctrlLost,
            StatusFlag::EcMismatch,
            StatusFlag::SteepCurve,
            StatusFlag::CurveInvalid,
            StatusFlag::GpuHot,
            StatusFlag::NvmeHot,
            StatusFlag::ReadbackBlind,
        ] {
            let mut m = Model::new();
            m.update(Event::Status(status_with_flag(flag)));
            let terminal = draw(&m);
            let header = row_text(&terminal, 0);
            let text = flag_text(flag);
            assert!(
                header.contains(text),
                "flag {flag:?} must render {text:?}; header was: {header:?}"
            );
        }
    }

    #[test]
    fn warning_severity_new_flags_are_yellow() {
        for flag in [
            StatusFlag::CurveInvalid,
            StatusFlag::EcMismatch,
            StatusFlag::FanctrlLost,
            StatusFlag::GpuHot,
            StatusFlag::NvmeHot,
        ] {
            let mut m = Model::new();
            m.update(Event::Status(status_with_flag(flag)));
            let terminal = draw(&m);
            let header = row_text(&terminal, 0);
            let text = flag_text(flag);
            let x = find_col(&header, text)
                .unwrap_or_else(|| panic!("{text:?} missing: {header:?}"))
                as u16;
            let cell = terminal.backend().buffer().cell((x, 0)).unwrap();
            assert_eq!(cell.fg, Color::Yellow, "flag {flag:?} must be yellow");
        }
    }

    #[test]
    fn info_severity_new_flags_are_gray() {
        for flag in [StatusFlag::SteepCurve, StatusFlag::ReadbackBlind] {
            let mut m = Model::new();
            m.update(Event::Status(status_with_flag(flag)));
            let terminal = draw(&m);
            let header = row_text(&terminal, 0);
            let text = flag_text(flag);
            let x = find_col(&header, text)
                .unwrap_or_else(|| panic!("{text:?} missing: {header:?}"))
                as u16;
            let cell = terminal.backend().buffer().cell((x, 0)).unwrap();
            assert_eq!(cell.fg, Color::Gray, "flag {flag:?} must be gray");
        }
    }

    /// Acceptance criteria, verbatim: a warning-severity flag must outrank
    /// an info one when both are present — specifically CURVE INVALID
    /// (warning) over STEEP CURVE (info).
    #[test]
    fn curve_invalid_outranks_steep_curve() {
        use crate::control::ControlStatus;
        let mut m = Model::new();
        m.update(Event::Status(ControlStatus {
            // Info flag listed FIRST in the Vec: only severity-first
            // sorting, not insertion order, can put CURVE INVALID ahead.
            flags: vec![StatusFlag::SteepCurve, StatusFlag::CurveInvalid],
            ..ControlStatus::default()
        }));
        let terminal = draw(&m);
        let header = row_text(&terminal, 0);
        let curve_invalid = find_col(&header, "CURVE INVALID").expect("CURVE INVALID must render");
        let steep_curve = find_col(&header, "STEEP CURVE").expect("STEEP CURVE must render");
        assert!(
            curve_invalid < steep_curve,
            "CURVE INVALID (warning) must outrank STEEP CURVE (info): {header:?}"
        );
    }

    // --- Task 15: calibration phase renders verbatim ---

    #[test]
    fn calib_wizard_renders_settle_phase_verbatim() {
        use crate::control::ControlStatus;
        use crate::control::controller::{CalibProgressLite, Mode};
        let mut m = Model::new();
        m.update(Event::Status(ControlStatus {
            mode: Mode::Calibrating,
            calib: Some(CalibProgressLite {
                phase: "settle".into(),
                step: 2,
                total: 10,
                needs_load: false,
                note: String::new(),
            }),
            ..ControlStatus::default()
        }));
        let text = all_text(&draw(&m));
        assert!(text.contains("calibration \u{2014} settle"), "text: {text}");
    }

    #[test]
    fn calib_wizard_renders_step_phase_verbatim() {
        use crate::control::ControlStatus;
        use crate::control::controller::{CalibProgressLite, Mode};
        let mut m = Model::new();
        m.update(Event::Status(ControlStatus {
            mode: Mode::Calibrating,
            calib: Some(CalibProgressLite {
                phase: "step".into(),
                step: 5,
                total: 8,
                needs_load: false,
                note: String::new(),
            }),
            ..ControlStatus::default()
        }));
        let text = all_text(&draw(&m));
        assert!(text.contains("calibration \u{2014} step"), "text: {text}");
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

    #[test]
    fn gpu_watts_chart_names_its_fixed_presentation_scale() {
        let text = all_text(&draw_size(&Model::new(), 120, 40));
        assert!(text.contains("watts (% of display scale)"), "text: {text}");
        assert!(!text.contains("watts (% of max)"), "text: {text}");
    }
}

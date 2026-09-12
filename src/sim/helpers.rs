//! Shared trace graders for the revision-4 closed-loop simulations.

const RELAY_WINDOW_S: f64 = 30.0 * 60.0;
const MIN_RELAY_PERIOD_S: f64 = 30.0;
const MAX_RELAY_PERIOD_S: f64 = 20.0 * 60.0;
const MIN_SUSTAINED_RELAY_S: f64 = RELAY_WINDOW_S / 3.0;
const TRACE_CADENCE_S: f64 = 1.0;

pub(super) fn band_residency_pct(errors: &[f64], band: f64) -> f64 {
    assert!(!errors.is_empty(), "band-residency bar has an empty series");
    let in_band = errors.iter().filter(|error| error.abs() <= band).count();
    100.0 * in_band as f64 / errors.len() as f64
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct KneeSample {
    pub time_s: f64,
    pub hot_error_c: f64,
    pub thermal_candidate_mhz: f64,
}

impl KneeSample {
    pub(super) const fn new(time_s: f64, hot_error_c: f64, thermal_candidate_mhz: f64) -> Self {
        Self {
            time_s,
            hot_error_c,
            thermal_candidate_mhz,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct KneeMeasurement {
    pub hot_start_s: f64,
    pub plateau_start_s: f64,
    pub crossing_s: f64,
    pub distance_to_knee: f64,
    pub minimum_hot_error_c: f64,
    pub crossing_deadline_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum KneeError {
    NoHotResponse,
    NoDownwardCrossing,
    NoSustainedHotError,
    InterruptedHotPlateau,
    NoNonIncreasingHotPlateau,
    InvalidGains,
}

/// Measures the first downward knee crossing after the response becomes hot.
/// An earlier upward crossing is deliberately outside the searched suffix.
pub(super) fn measure_downward_knee(
    samples: &[KneeSample],
    knee_mhz: f64,
    kc_mhz_per_c: f64,
    ti_s: f64,
    pi_period_s: f64,
) -> Result<KneeMeasurement, KneeError> {
    if !(kc_mhz_per_c.is_finite() && kc_mhz_per_c > 0.0 && ti_s.is_finite() && ti_s > 0.0) {
        return Err(KneeError::InvalidGains);
    }
    let hot_index = samples
        .iter()
        .position(|sample| sample.hot_error_c.is_finite() && sample.hot_error_c > 0.0)
        .ok_or(KneeError::NoHotResponse)?;
    let hot = samples[hot_index];
    let hot_distance_to_knee = (hot.thermal_candidate_mhz - knee_mhz).max(0.0);
    let crossing_index = if hot_distance_to_knee == 0.0 {
        hot_index
    } else {
        (hot_index + 1..samples.len())
            .find(|&index| {
                samples[index - 1].thermal_candidate_mhz > knee_mhz
                    && samples[index].thermal_candidate_mhz <= knee_mhz
            })
            .ok_or(KneeError::NoDownwardCrossing)?
    };
    if samples[hot_index..=crossing_index]
        .iter()
        .any(|sample| !sample.hot_error_c.is_finite() || sample.hot_error_c <= 0.0)
    {
        return Err(KneeError::InterruptedHotPlateau);
    }
    let plateau_index = if hot_distance_to_knee == 0.0 {
        hot_index
    } else {
        (hot_index..=crossing_index)
            .find(|&start| {
                samples[crossing_index].time_s - samples[start].time_s >= pi_period_s
                    && samples[start..=crossing_index]
                        .windows(2)
                        .all(|pair| pair[1].hot_error_c <= pair[0].hot_error_c)
            })
            .ok_or(KneeError::NoNonIncreasingHotPlateau)?
    };
    let plateau = samples[plateau_index];
    let distance_to_knee = (plateau.thermal_candidate_mhz - knee_mhz).max(0.0);
    let minimum_hot_error_c = samples[plateau_index..=crossing_index]
        .iter()
        .map(|sample| sample.hot_error_c)
        .filter(|error| error.is_finite() && *error > 0.0)
        .fold(f64::INFINITY, f64::min);
    if !minimum_hot_error_c.is_finite() || minimum_hot_error_c <= 0.0 {
        return Err(KneeError::NoSustainedHotError);
    }
    let crossing_deadline_s = if distance_to_knee == 0.0 {
        plateau.time_s
    } else {
        plateau.time_s
            + distance_to_knee / (kc_mhz_per_c / ti_s * minimum_hot_error_c)
            + pi_period_s.max(0.0)
    };
    Ok(KneeMeasurement {
        hot_start_s: hot.time_s,
        plateau_start_s: samples[plateau_index].time_s,
        crossing_s: samples[crossing_index].time_s,
        distance_to_knee,
        minimum_hot_error_c,
        crossing_deadline_s,
    })
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct RelaySample {
    pub time_s: f64,
    pub cpu_cap_w: f64,
    pub gpu_cap_mhz: f64,
    pub fan_rpm: f64,
}

impl RelaySample {
    pub(super) const fn new(time_s: f64, cpu_cap_w: f64, gpu_cap_mhz: f64, fan_rpm: f64) -> Self {
        Self {
            time_s,
            cpu_cap_w,
            gpu_cap_mhz,
            fan_rpm,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum RelaySignal {
    CpuWatts,
    GpuMhz,
    FanRpm,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct RelayFinding {
    pub signal: RelaySignal,
    pub window_start_s: f64,
    pub period_s: f64,
    pub peak_to_peak: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub(super) struct RelayGrade {
    pub findings: Vec<RelayFinding>,
    pub window_starts_s: Vec<f64>,
}

#[derive(Debug, Clone, Copy)]
pub(super) struct RelayForcingReference<'a> {
    pub samples: &'a [RelaySample],
    pub period_s: f64,
    pub scored_active_from_s: f64,
    pub reference_active_from_s: f64,
    pub steady_after_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum RelayGradeError {
    EmptyTrace,
    ShortTrace { span_s: f64 },
    EmptyReference,
    InvalidReferencePeriod,
    InvalidReferenceTiming,
    InvalidTraceTimestamps,
    InvalidReferenceTimestamps,
    SparseTrace { gap_s: f64 },
    SparseReference { gap_s: f64 },
    ShortReference { span_s: f64 },
    IncompleteReferenceCoverage,
    UnshiftedReference,
    ClonedReference,
    SelfFittedReference,
}

fn validate_timestamps(samples: &[RelaySample], is_reference: bool) -> Result<(), RelayGradeError> {
    if samples.iter().any(|sample| !sample.time_s.is_finite())
        || samples
            .windows(2)
            .any(|pair| pair[1].time_s <= pair[0].time_s)
    {
        return Err(if is_reference {
            RelayGradeError::InvalidReferenceTimestamps
        } else {
            RelayGradeError::InvalidTraceTimestamps
        });
    }
    if let Some(gap_s) = samples
        .windows(2)
        .map(|pair| pair[1].time_s - pair[0].time_s)
        .find(|gap| *gap > TRACE_CADENCE_S + 1e-6)
    {
        return Err(if is_reference {
            RelayGradeError::SparseReference { gap_s }
        } else {
            RelayGradeError::SparseTrace { gap_s }
        });
    }
    Ok(())
}

fn signal_value(sample: &RelaySample, signal: RelaySignal) -> f64 {
    match signal {
        RelaySignal::CpuWatts => sample.cpu_cap_w,
        RelaySignal::GpuMhz => sample.gpu_cap_mhz,
        RelaySignal::FanRpm => sample.fan_rpm,
    }
}

fn relay_threshold(signal: RelaySignal) -> f64 {
    match signal {
        RelaySignal::CpuWatts => 4.0,
        RelaySignal::GpuMhz => 150.0,
        RelaySignal::FanRpm => 100.0,
    }
}

struct RelayReferenceModel {
    period_s: f64,
    scored_active_from_s: f64,
    steady_after_s: f64,
    baseline: f64,
    phases: Vec<Option<f64>>,
    transient: Vec<(f64, f64)>,
}

impl RelayReferenceModel {
    fn value_at(&self, time_s: f64) -> Option<f64> {
        if time_s < self.scored_active_from_s {
            return None;
        }
        let elapsed_s = time_s - self.scored_active_from_s;
        if elapsed_s < self.steady_after_s {
            return self
                .transient
                .binary_search_by(|(candidate, _)| candidate.total_cmp(&elapsed_s))
                .ok()
                .map(|index| self.transient[index].1);
        }
        let phase = (elapsed_s.rem_euclid(self.period_s).round() as usize) % self.phases.len();
        self.phases[phase]
    }
}

/// Finds sustained alternating excursions in every complete 30-minute
/// window. The detector derives periods from consecutive high/low transition
/// intervals, so it has no baked-in candidate period.
pub(super) fn relay_findings(
    samples: &[RelaySample],
    forcing_reference: Option<RelayForcingReference<'_>>,
) -> Result<RelayGrade, RelayGradeError> {
    let Some(first) = samples.first() else {
        return Err(RelayGradeError::EmptyTrace);
    };
    validate_timestamps(samples, false)?;
    let last = samples.last().expect("non-empty trace");
    let span_s = last.time_s - first.time_s;
    if span_s < RELAY_WINDOW_S - 1.0 {
        return Err(RelayGradeError::ShortTrace { span_s });
    }
    if let Some(reference) = forcing_reference {
        if reference.samples.is_empty() {
            return Err(RelayGradeError::EmptyReference);
        }
        validate_timestamps(reference.samples, true)?;
        let reference_span_s =
            reference.samples.last().unwrap().time_s - reference.samples.first().unwrap().time_s;
        if reference_span_s < RELAY_WINDOW_S - 1.0 {
            return Err(RelayGradeError::ShortReference {
                span_s: reference_span_s,
            });
        }
        if !reference.period_s.is_finite() || reference.period_s < 1.0 {
            return Err(RelayGradeError::InvalidReferencePeriod);
        }
        if !reference.scored_active_from_s.is_finite()
            || !reference.reference_active_from_s.is_finite()
            || !reference.steady_after_s.is_finite()
            || reference.steady_after_s < 0.0
        {
            return Err(RelayGradeError::InvalidReferenceTiming);
        }
        let phase_shift_s = (reference.scored_active_from_s - reference.reference_active_from_s)
            .rem_euclid(reference.period_s);
        if phase_shift_s <= 1e-6 || reference.period_s - phase_shift_s <= 1e-6 {
            return Err(RelayGradeError::UnshiftedReference);
        }
        if std::ptr::eq(samples.as_ptr(), reference.samples.as_ptr()) {
            return Err(RelayGradeError::SelfFittedReference);
        }
        if samples == reference.samples {
            return Err(RelayGradeError::ClonedReference);
        }
        let reference_first_s = reference.samples.first().unwrap().time_s;
        let reference_last_s = reference.samples.last().unwrap().time_s;
        if reference_first_s > reference.reference_active_from_s
            || reference_last_s
                < reference.reference_active_from_s + reference.steady_after_s + reference.period_s
                    - TRACE_CADENCE_S
        {
            return Err(RelayGradeError::IncompleteReferenceCoverage);
        }
    }
    let starts: Vec<_> = samples
        .iter()
        .map(|sample| sample.time_s)
        .take_while(|start| *start + RELAY_WINDOW_S - 1.0 <= last.time_s)
        .collect();
    let reference_models: Vec<_> = [
        RelaySignal::CpuWatts,
        RelaySignal::GpuMhz,
        RelaySignal::FanRpm,
    ]
    .into_iter()
    .map(|signal| {
        forcing_reference.map(|reference| {
            let bins = reference.period_s.round() as usize;
            let reference_steady_from_s =
                reference.reference_active_from_s + reference.steady_after_s;
            let steady: Vec<_> = reference
                .samples
                .iter()
                .filter(|sample| sample.time_s >= reference_steady_from_s)
                .collect();
            let steady_mean = steady
                .iter()
                .map(|sample| signal_value(sample, signal))
                .sum::<f64>()
                / steady.len().max(1) as f64;
            let baseline_samples: Vec<_> = reference
                .samples
                .iter()
                .filter(|sample| {
                    sample.time_s < reference.reference_active_from_s
                        && sample.time_s >= reference.reference_active_from_s - 60.0
                })
                .collect();
            let baseline = if baseline_samples.is_empty() {
                steady_mean
            } else {
                baseline_samples
                    .iter()
                    .map(|sample| signal_value(sample, signal))
                    .sum::<f64>()
                    / baseline_samples.len() as f64
            };
            let mut phase_sums = vec![(0.0, 0usize); bins];
            for sample in steady {
                let phase = ((sample.time_s - reference.reference_active_from_s)
                    .rem_euclid(reference.period_s)
                    .round() as usize)
                    % bins;
                phase_sums[phase].0 += signal_value(sample, signal);
                phase_sums[phase].1 += 1;
            }
            RelayReferenceModel {
                period_s: reference.period_s,
                scored_active_from_s: reference.scored_active_from_s,
                steady_after_s: reference.steady_after_s,
                baseline,
                phases: phase_sums
                    .into_iter()
                    .map(|(sum, count)| (count > 0).then_some(sum / count as f64))
                    .collect(),
                transient: reference
                    .samples
                    .iter()
                    .filter(|sample| {
                        sample.time_s >= reference.reference_active_from_s
                            && sample.time_s < reference_steady_from_s
                    })
                    .map(|sample| {
                        (
                            sample.time_s - reference.reference_active_from_s,
                            signal_value(sample, signal),
                        )
                    })
                    .collect(),
            }
        })
    })
    .collect();

    let mut findings = Vec::new();
    for window_start_s in starts.iter().copied() {
        let window_end_s = window_start_s + RELAY_WINDOW_S;
        let window: Vec<_> = samples
            .iter()
            .filter(|sample| sample.time_s >= window_start_s && sample.time_s < window_end_s)
            .collect();
        for (signal_index, signal) in [
            RelaySignal::CpuWatts,
            RelaySignal::GpuMhz,
            RelaySignal::FanRpm,
        ]
        .into_iter()
        .enumerate()
        {
            let mut values: Vec<_> = window
                .iter()
                .map(|sample| signal_value(sample, signal))
                .collect();
            // Remove only the response measured in a separate forcing-only
            // trace. During forcing entry use its timestamp-matched
            // transient; after convergence use its phase-keyed ensemble.
            // The tested trace never contributes to the reference model.
            if let Some(reference) = &reference_models[signal_index] {
                for (sample, value) in window.iter().zip(&mut values) {
                    if let Some(reference_value) = reference.value_at(sample.time_s) {
                        *value -= reference_value - reference.baseline;
                    }
                }
            }
            let min = values.iter().copied().fold(f64::INFINITY, f64::min);
            let max = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            let peak_to_peak = max - min;
            if !peak_to_peak.is_finite() || peak_to_peak <= relay_threshold(signal) {
                continue;
            }
            let midpoint = (min + max) / 2.0;
            let deadband = peak_to_peak * 0.15;
            let mut last_side = 0i8;
            let mut transitions = Vec::new();
            for (sample, value) in window.iter().zip(&values) {
                let value = *value;
                let side = if value >= midpoint + deadband {
                    1
                } else if value <= midpoint - deadband {
                    -1
                } else {
                    0
                };
                if side != 0 && last_side != 0 && side != last_side {
                    let outside_transition_chatter = transitions
                        .last()
                        .is_none_or(|last| sample.time_s - last >= MIN_RELAY_PERIOD_S / 2.0);
                    if outside_transition_chatter {
                        transitions.push(sample.time_s);
                    }
                }
                if side != 0 {
                    last_side = side;
                }
            }
            if transitions.len() < 2 {
                continue;
            }
            if transitions.last().unwrap() - transitions.first().unwrap() < MIN_SUSTAINED_RELAY_S {
                continue;
            }
            let half_periods: Vec<_> = transitions
                .windows(2)
                .map(|pair| pair[1] - pair[0])
                .filter(|duration| *duration > 0.0)
                .collect();
            let mut sorted_half_periods = half_periods.clone();
            sorted_half_periods.sort_by(f64::total_cmp);
            let median_half_period_s = sorted_half_periods[sorted_half_periods.len() / 2];
            let regular_intervals = half_periods
                .iter()
                .filter(|duration| {
                    (**duration - median_half_period_s).abs() <= 0.25 * median_half_period_s
                })
                .count();
            if half_periods.len() > 1 && regular_intervals * 4 < half_periods.len() * 3 {
                continue;
            }
            let period_s = 2.0 * median_half_period_s;
            if (MIN_RELAY_PERIOD_S..=MAX_RELAY_PERIOD_S).contains(&period_s) {
                let lag = period_s.round() as usize;
                if lag >= values.len() {
                    continue;
                }
                let mean = values.iter().sum::<f64>() / values.len() as f64;
                let (covariance, energy_now, energy_lagged) = values[..values.len() - lag]
                    .iter()
                    .zip(&values[lag..])
                    .fold((0.0, 0.0, 0.0), |(cov, left, right), (a, b)| {
                        let a = *a - mean;
                        let b = *b - mean;
                        (cov + a * b, left + a * a, right + b * b)
                    });
                let phase_correlation = covariance / (energy_now * energy_lagged).sqrt();
                if !phase_correlation.is_finite() || phase_correlation < 0.5 {
                    continue;
                }
                findings.push(RelayFinding {
                    signal,
                    window_start_s,
                    period_s,
                    peak_to_peak,
                });
            }
        }
    }
    Ok(RelayGrade {
        findings,
        window_starts_s: starts,
    })
}

/// First sample at or after `origin_s` beginning a continuously in-band
/// interval of at least `sustain_s`. The returned timestamp is the timing
/// origin of that successful interval, rather than its end.
pub(super) fn first_sustained_in_band(
    samples: &[(f64, f64)],
    origin_s: f64,
    band: f64,
    sustain_s: f64,
    max_gap_s: f64,
) -> Option<f64> {
    let mut start = None;
    let mut previous_time = None;
    for &(time_s, error) in samples.iter().filter(|(time_s, _)| *time_s >= origin_s) {
        let continuous = previous_time
            .is_none_or(|previous| time_s > previous && time_s - previous <= max_gap_s);
        if error.abs() <= band {
            if !continuous {
                start = None;
            }
            let start_s = *start.get_or_insert(time_s);
            if time_s - start_s >= sustain_s {
                return Some(start_s);
            }
        } else {
            start = None;
        }
        previous_time = Some(time_s);
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn band_residency_counts_ninety_percent_exactly() {
        let mut errors = vec![0.0; 9];
        errors.push(151.0);
        assert_eq!(band_residency_pct(&errors, 150.0), 90.0);
    }

    #[test]
    fn sustained_band_reports_the_start_not_the_end_of_the_qualifying_interval() {
        let samples = [
            (0.0, 2.0),
            (1.0, 0.5),
            (2.0, 0.4),
            (3.0, 1.2),
            (4.0, 0.3),
            (5.0, 0.2),
            (6.0, 0.1),
        ];
        assert_eq!(
            first_sustained_in_band(&samples, 2.0, 1.0, 2.0, 1.0),
            Some(4.0)
        );
    }

    #[test]
    fn sustained_band_rejects_a_timestamp_gap() {
        let samples = [(0.0, 0.2), (1.0, 0.1), (10.0, 0.1), (11.0, 0.2)];
        assert_eq!(first_sustained_in_band(&samples, 0.0, 1.0, 2.0, 1.0), None);
    }

    #[test]
    fn downward_knee_ignores_the_earlier_upward_crossing() {
        let trace = [
            KneeSample::new(0.0, -1.0, 2100.0),
            KneeSample::new(5.0, -0.5, 2200.0),
            KneeSample::new(10.0, 3.0, 2300.0),
            KneeSample::new(15.0, 2.5, 2150.0),
            KneeSample::new(20.0, 2.0, 2140.0),
        ];
        let measured = measure_downward_knee(&trace, 2143.0, 2.1, 15.0, 5.0)
            .expect("hot response has a downward crossing");
        assert_eq!(measured.hot_start_s, 10.0);
        assert_eq!(measured.crossing_s, 20.0);
        assert_eq!(measured.distance_to_knee, 157.0);
    }

    #[test]
    fn start_below_knee_uses_zero_distance_and_hot_start_as_crossing() {
        let trace = [
            KneeSample::new(2.0, -0.1, 2100.0),
            KneeSample::new(3.0, 2.0, 2100.0),
            KneeSample::new(4.0, 1.5, 2090.0),
        ];
        let measured = measure_downward_knee(&trace, 2143.0, 2.1, 15.0, 5.0)
            .expect("hot response begins below the knee");
        assert_eq!(measured.distance_to_knee, 0.0);
        assert_eq!(measured.crossing_s, 3.0);
        assert_eq!(measured.crossing_deadline_s, 3.0);
    }

    #[test]
    fn no_hot_response_has_no_finite_crossing_claim() {
        let trace = [
            KneeSample::new(0.0, 0.0, 2300.0),
            KneeSample::new(1.0, -1.0, 2100.0),
        ];
        assert_eq!(
            measure_downward_knee(&trace, 2143.0, 2.1, 15.0, 5.0),
            Err(KneeError::NoHotResponse)
        );
    }

    #[test]
    fn downward_knee_rejects_an_interrupted_hot_plateau() {
        let trace = [
            KneeSample::new(0.0, -1.0, 2300.0),
            KneeSample::new(5.0, 4.0, 2280.0),
            KneeSample::new(10.0, 0.0, 2200.0),
            KneeSample::new(15.0, 3.0, 2140.0),
        ];
        assert_eq!(
            measure_downward_knee(&trace, 2143.0, 2.1, 15.0, 5.0),
            Err(KneeError::InterruptedHotPlateau)
        );
    }

    #[test]
    fn downward_knee_rejects_an_increasing_hot_plateau() {
        let trace = [
            KneeSample::new(0.0, -1.0, 2300.0),
            KneeSample::new(5.0, 2.0, 2280.0),
            KneeSample::new(10.0, 3.0, 2200.0),
            KneeSample::new(15.0, 4.0, 2140.0),
        ];
        assert_eq!(
            measure_downward_knee(&trace, 2143.0, 2.1, 15.0, 5.0),
            Err(KneeError::NoNonIncreasingHotPlateau)
        );
    }

    #[test]
    fn downward_knee_times_distance_and_deadline_from_the_validated_plateau() {
        let trace = [
            KneeSample::new(0.0, 4.0, 2400.0),
            KneeSample::new(5.0, 5.0, 2300.0),
            KneeSample::new(10.0, 4.0, 2200.0),
            KneeSample::new(15.0, 3.0, 2140.0),
        ];
        let measured = measure_downward_knee(&trace, 2143.0, 2.1, 15.0, 5.0)
            .expect("later non-increasing plateau reaches the knee");
        assert_eq!(measured.hot_start_s, 0.0);
        assert_eq!(measured.plateau_start_s, 5.0);
        assert_eq!(measured.distance_to_knee, 157.0);
        assert_eq!(measured.minimum_hot_error_c, 3.0);
        assert_eq!(measured.crossing_deadline_s, 5.0 + 157.0 / 0.42 + 5.0);
    }

    fn square_wave(period_s: u64, seconds: u64, amplitude: f64) -> Vec<RelaySample> {
        (0..seconds)
            .map(|t| {
                let high = (t / (period_s / 2)).is_multiple_of(2);
                let sign = if high { 1.0 } else { -1.0 };
                RelaySample::new(
                    t as f64,
                    30.0 + sign * amplitude,
                    2000.0 + sign * amplitude * 50.0,
                    2500.0 + sign * amplitude * 40.0,
                )
            })
            .collect()
    }

    #[test]
    fn relay_grader_rejects_30_300_and_1200_second_cycles() {
        for period in [30, 300, 1200] {
            let trace = square_wave(period, 3600, 3.0);
            let findings = relay_findings(&trace, None)
                .expect("complete relay trace")
                .findings;
            assert!(
                !findings.is_empty(),
                "{period}s relay escaped the 30-minute window grader"
            );
            assert!(
                findings.iter().any(|finding| finding.window_start_s < 1.0)
                    && findings
                        .iter()
                        .any(|finding| finding.window_start_s >= 1800.0),
                "{period}s relay was not rejected in every non-overlapping 30-minute window: {findings:?}"
            );
        }
    }

    #[test]
    fn relay_grader_passes_bounded_noise_and_only_the_declared_load_period() {
        let noise: Vec<_> = (0..1800)
            .map(|t| {
                RelaySample::new(
                    t as f64,
                    30.0 + (t % 7) as f64 * 0.1,
                    2000.0,
                    2500.0 + (t % 11) as f64,
                )
            })
            .collect();
        assert!(relay_findings(&noise, None).unwrap().findings.is_empty());

        let phase_square = |origin: u64| {
            (0_u64..1800)
                .map(|t| {
                    let high = ((t.saturating_sub(origin)) / 150).is_multiple_of(2);
                    let sign = if high { 1.0 } else { -1.0 };
                    RelaySample::new(
                        t as f64,
                        30.0 + sign * 3.0,
                        2000.0 + sign * 150.0,
                        2500.0 + sign * 120.0,
                    )
                })
                .collect::<Vec<_>>()
        };
        let imposed = phase_square(300);
        let forcing_only = phase_square(225);
        let reference = RelayForcingReference {
            samples: &forcing_only,
            period_s: 300.0,
            scored_active_from_s: 300.0,
            reference_active_from_s: 225.0,
            steady_after_s: 600.0,
        };
        assert!(
            relay_findings(&imposed, Some(reference))
                .unwrap()
                .findings
                .is_empty()
        );
        assert!(!relay_findings(&imposed, None).unwrap().findings.is_empty());
        assert_eq!(
            relay_findings(
                &imposed,
                Some(RelayForcingReference {
                    samples: &imposed,
                    ..reference
                })
            ),
            Err(RelayGradeError::SelfFittedReference)
        );
    }

    #[test]
    fn full_amplitude_seeded_fan_noise_is_not_a_periodic_relay() {
        let mut state = 0xEB9_u32;
        let noise: Vec<_> = (0..3600)
            .map(|t| {
                state ^= state << 13;
                state ^= state >> 17;
                state ^= state << 5;
                let unit = f64::from(state) / f64::from(u32::MAX);
                RelaySample::new(t as f64, 30.0, 2000.0, 2500.0 - 90.0 + 180.0 * unit)
            })
            .collect();
        assert!(relay_findings(&noise, None).unwrap().findings.is_empty());
    }

    #[test]
    fn relay_grader_rejects_empty_short_and_grades_every_origin() {
        assert_eq!(relay_findings(&[], None), Err(RelayGradeError::EmptyTrace));
        assert!(matches!(
            relay_findings(&square_wave(300, 1799, 3.0), None),
            Err(RelayGradeError::ShortTrace { .. })
        ));
        let grade = relay_findings(&square_wave(300, 3600, 3.0), None).unwrap();
        assert_eq!(grade.window_starts_s.len(), 1801);
        assert_eq!(grade.window_starts_s.first(), Some(&0.0));
        assert_eq!(grade.window_starts_s.last(), Some(&1800.0));
        assert!(
            grade
                .findings
                .iter()
                .any(|finding| finding.window_start_s == 1800.0),
            "late complete window was not graded: {grade:?}"
        );
    }

    #[test]
    fn relay_grader_rejects_sparse_or_invalid_scored_timestamps() {
        let two_points = [
            RelaySample::new(0.0, 30.0, 2000.0, 2500.0),
            RelaySample::new(1800.0, 30.0, 2000.0, 2500.0),
        ];
        assert!(matches!(
            relay_findings(&two_points, None),
            Err(RelayGradeError::SparseTrace { .. })
        ));

        let mut late_gap = square_wave(300, 3600, 3.0);
        late_gap.remove(3000);
        assert!(matches!(
            relay_findings(&late_gap, None),
            Err(RelayGradeError::SparseTrace { .. })
        ));

        let mut non_finite = square_wave(300, 1801, 3.0);
        non_finite[900].time_s = f64::NAN;
        assert_eq!(
            relay_findings(&non_finite, None),
            Err(RelayGradeError::InvalidTraceTimestamps)
        );
    }

    #[test]
    fn relay_grader_rejects_sparse_or_invalid_reference_coverage() {
        let scored = square_wave(300, 3600, 3.0);
        let two_points = [
            RelaySample::new(0.0, 30.0, 2000.0, 2500.0),
            RelaySample::new(1800.0, 30.0, 2000.0, 2500.0),
        ];
        let reference = |samples| RelayForcingReference {
            samples,
            period_s: 300.0,
            scored_active_from_s: 300.0,
            reference_active_from_s: 225.0,
            steady_after_s: 600.0,
        };
        assert!(matches!(
            relay_findings(&scored, Some(reference(&two_points))),
            Err(RelayGradeError::SparseReference { .. })
        ));

        let mut late_gap = square_wave(300, 3600, 3.0);
        late_gap.remove(3000);
        assert!(matches!(
            relay_findings(&scored, Some(reference(&late_gap))),
            Err(RelayGradeError::SparseReference { .. })
        ));

        let mut duplicate = square_wave(300, 3600, 3.0);
        duplicate[901].time_s = duplicate[900].time_s;
        assert_eq!(
            relay_findings(&scored, Some(reference(&duplicate))),
            Err(RelayGradeError::InvalidReferenceTimestamps)
        );
    }

    #[test]
    fn forcing_reference_requires_shifted_independent_provenance() {
        let scored = square_wave(300, 3600, 3.0);
        let cloned = scored.clone();
        let shifted = RelayForcingReference {
            samples: &cloned,
            period_s: 300.0,
            scored_active_from_s: 300.0,
            reference_active_from_s: 225.0,
            steady_after_s: 600.0,
        };
        assert_eq!(
            relay_findings(&scored, Some(shifted)),
            Err(RelayGradeError::ClonedReference)
        );
        assert_eq!(
            relay_findings(
                &scored,
                Some(RelayForcingReference {
                    reference_active_from_s: 300.0,
                    ..shifted
                })
            ),
            Err(RelayGradeError::UnshiftedReference)
        );
    }

    #[test]
    fn phase_residual_does_not_exempt_extra_same_or_near_period_relays() {
        const SCORED_ORIGIN: u64 = 300;
        const REFERENCE_ORIGIN: u64 = 225;
        let forcing_only: Vec<_> = (0_u64..3600)
            .map(|t| {
                let forcing = if ((t.saturating_sub(REFERENCE_ORIGIN)) / 150).is_multiple_of(2) {
                    3.0
                } else {
                    -3.0
                };
                RelaySample::new(t as f64, 30.0 + forcing, 2000.0, 2500.0)
            })
            .collect();
        let reference = RelayForcingReference {
            samples: &forcing_only,
            period_s: 300.0,
            scored_active_from_s: SCORED_ORIGIN as f64,
            reference_active_from_s: REFERENCE_ORIGIN as f64,
            steady_after_s: 600.0,
        };
        for extra_period in [300_u64, 307] {
            let trace: Vec<_> = (0_u64..3600)
                .map(|t| {
                    let forcing = if ((t.saturating_sub(SCORED_ORIGIN)) / 150).is_multiple_of(2) {
                        3.0
                    } else {
                        -3.0
                    };
                    let shifted_t = t + extra_period / 4;
                    let extra = if (shifted_t / (extra_period / 2)).is_multiple_of(2) {
                        3.0
                    } else {
                        -3.0
                    };
                    RelaySample::new(t as f64, 30.0 + forcing + extra, 2000.0, 2500.0)
                })
                .collect();
            let grade = relay_findings(&trace, Some(reference)).unwrap();
            assert!(
                !grade.findings.is_empty(),
                "extra {extra_period}s relay escaped after declared-forcing residualization"
            );
        }
    }

    #[test]
    fn a_short_step_transient_is_not_a_sustained_relay() {
        let trace: Vec<_> = (0_u64..1800)
            .map(|t| {
                let fan = if t < 200 {
                    2500.0
                        + if (t / 24).is_multiple_of(2) {
                            800.0
                        } else {
                            -800.0
                        }
                } else {
                    2500.0
                };
                RelaySample::new(t as f64, 30.0, 2000.0, fan)
            })
            .collect();
        assert!(relay_findings(&trace, None).unwrap().findings.is_empty());
    }

    #[test]
    fn irregular_nonperiodic_excursions_are_not_reported_as_a_relay_period() {
        let transition_times = [100_u64, 160, 900, 1450];
        let trace: Vec<_> = (0_u64..1800)
            .map(|t| {
                let transitions = transition_times.iter().filter(|time| **time <= t).count();
                let fan = if transitions.is_multiple_of(2) {
                    2300.0
                } else {
                    2700.0
                };
                RelaySample::new(t as f64, 30.0, 2000.0, fan)
            })
            .collect();
        assert!(relay_findings(&trace, None).unwrap().findings.is_empty());
    }

    #[test]
    fn cap_relay_is_rejected_even_when_fan_stays_inside_its_band() {
        let mut trace = square_wave(300, 1800, 3.0);
        for row in &mut trace {
            row.fan_rpm = 2500.0;
        }
        let findings = relay_findings(&trace, None).unwrap().findings;
        assert!(
            findings
                .iter()
                .any(|finding| finding.signal == RelaySignal::CpuWatts)
        );
    }
}

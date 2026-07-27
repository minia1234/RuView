//! Robust per-link, per-subcarrier empty-room baseline and environmental drift tracking.

use crate::parser::CsiFrame;
use serde::{Deserialize, Serialize};
use std::time::{SystemTime, UNIX_EPOCH};

const DEFAULT_TARGET_FRAMES: usize = 600; // 30 s at 20 Hz
const MIN_BASELINE_FRAMES: usize = 40;
const MAX_BASELINE_FRAMES: usize = 1_200;
const EPSILON: f32 = 1.0e-5;

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BaselineConfig {
    pub target_frames: usize,
    pub min_stable_carrier_ratio: f32,
    pub max_amplitude_mad_ratio: f32,
    pub max_phase_mad_rad: f32,
    pub minimum_quality: f32,
    pub drift_warning_threshold: f32,
    pub drift_recalibration_threshold: f32,
}

impl Default for BaselineConfig {
    fn default() -> Self {
        Self {
            target_frames: DEFAULT_TARGET_FRAMES,
            min_stable_carrier_ratio: 0.55,
            max_amplitude_mad_ratio: 0.35,
            max_phase_mad_rad: 1.20,
            minimum_quality: 0.65,
            drift_warning_threshold: 0.35,
            drift_recalibration_threshold: 0.62,
        }
    }
}

impl BaselineConfig {
    pub fn normalized(mut self) -> Self {
        self.target_frames = self
            .target_frames
            .clamp(MIN_BASELINE_FRAMES, MAX_BASELINE_FRAMES);
        self.min_stable_carrier_ratio = self.min_stable_carrier_ratio.clamp(0.1, 1.0);
        self.max_amplitude_mad_ratio = self.max_amplitude_mad_ratio.clamp(0.01, 2.0);
        self.max_phase_mad_rad = self.max_phase_mad_rad.clamp(0.05, std::f32::consts::PI);
        self.minimum_quality = self.minimum_quality.clamp(0.05, 1.0);
        self.drift_warning_threshold = self.drift_warning_threshold.clamp(0.05, 2.0);
        self.drift_recalibration_threshold = self
            .drift_recalibration_threshold
            .max(self.drift_warning_threshold + 0.05)
            .clamp(0.1, 3.0);
        self
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum BaselineInvalidationReason {
    ManualReset,
    ChannelChanged,
    NodeMoved,
    StationMoved,
    LayoutChanged,
    RouterReboot,
    FirmwareChanged,
    CarrierShapeChanged,
    DriftExceeded,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct BaselineSummary {
    pub calibration_id: Option<String>,
    pub collecting: bool,
    pub ready: bool,
    pub frames_collected: usize,
    pub target_frames: usize,
    pub channel: Option<u8>,
    pub subcarrier_count: usize,
    pub stable_subcarriers: usize,
    pub unstable_subcarriers: Vec<usize>,
    pub stable_carrier_ratio: f32,
    pub finite_sample_ratio: f32,
    pub packet_quality: f32,
    pub phase_stability: f32,
    pub quality: f32,
    pub drift_index: f32,
    pub drift_warning: bool,
    pub recalibration_required: bool,
    pub invalidated_at_ms: Option<u64>,
    pub invalidation_reason: Option<BaselineInvalidationReason>,
    pub created_at_ms: Option<u64>,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct LinkObservation {
    pub amplitude_score: f32,
    pub phase_score: f32,
    pub rssi_score: f32,
    pub motion_score: f32,
    pub combined: f32,
    pub drift_index: f32,
    pub stable_carriers_used: usize,
    pub baseline_ready: bool,
}

#[derive(Clone, Debug)]
struct CompletedBaseline {
    calibration_id: String,
    channel: u8,
    n_subcarriers: usize,
    amplitude_median: Vec<f32>,
    amplitude_mad: Vec<f32>,
    phase_center: Vec<f32>,
    phase_mad: Vec<f32>,
    stable_mask: Vec<bool>,
    rssi_median: f32,
    rssi_mad: f32,
    quality: f32,
    finite_ratio: f32,
    packet_quality: f32,
    phase_stability: f32,
    created_at_ms: u64,
}

pub struct BaselineEngine {
    config: BaselineConfig,
    channel: Option<u8>,
    n_subcarriers: usize,
    amplitude_samples: Vec<Vec<f32>>,
    phase_samples: Vec<Vec<f32>>,
    rssi_samples: Vec<f32>,
    accepted_frames: usize,
    total_values: u64,
    finite_values: u64,
    packet_quality_ema: f32,
    completed: Option<CompletedBaseline>,
    invalidated_at_ms: Option<u64>,
    invalidation_reason: Option<BaselineInvalidationReason>,
    drift_index: f32,
    drift_warning: bool,
    recalibration_required: bool,
}

impl Default for BaselineEngine {
    fn default() -> Self {
        Self::new(BaselineConfig::default())
    }
}

impl BaselineEngine {
    pub fn new(config: BaselineConfig) -> Self {
        Self {
            config: config.normalized(),
            channel: None,
            n_subcarriers: 0,
            amplitude_samples: Vec::new(),
            phase_samples: Vec::new(),
            rssi_samples: Vec::new(),
            accepted_frames: 0,
            total_values: 0,
            finite_values: 0,
            packet_quality_ema: 1.0,
            completed: None,
            invalidated_at_ms: None,
            invalidation_reason: None,
            drift_index: 0.0,
            drift_warning: false,
            recalibration_required: false,
        }
    }

    pub fn ingest(
        &mut self,
        frame: &CsiFrame,
        packet_quality: f32,
        stationarity_score: f32,
        motion_score: f32,
    ) -> LinkObservation {
        if self.channel.is_some_and(|channel| channel != frame.channel) {
            self.invalidate(BaselineInvalidationReason::ChannelChanged);
        }
        if self.n_subcarriers != 0 && self.n_subcarriers != frame.amplitudes.len() {
            self.invalidate(BaselineInvalidationReason::CarrierShapeChanged);
        }
        if self.channel.is_none() {
            self.channel = Some(frame.channel);
        }
        if self.n_subcarriers == 0 {
            self.prepare_carriers(frame.amplitudes.len());
        }

        let packet_quality = packet_quality.clamp(0.0, 1.0);
        self.packet_quality_ema = self.packet_quality_ema * 0.97 + packet_quality * 0.03;

        if self.completed.is_none() {
            self.collect_frame(frame, stationarity_score);
            if self.accepted_frames >= self.config.target_frames {
                self.finalize();
            }
        }

        let Some(baseline) = &self.completed else {
            return LinkObservation {
                motion_score,
                baseline_ready: false,
                ..LinkObservation::default()
            };
        };

        let mut amplitude_total = 0.0f32;
        let mut phase_total = 0.0f32;
        let mut used = 0usize;
        for index in 0..baseline.n_subcarriers.min(frame.amplitudes.len()) {
            if !baseline.stable_mask[index] {
                continue;
            }
            let amplitude = frame.amplitudes[index];
            let phase = frame.phases.get(index).copied().unwrap_or(0.0);
            if !amplitude.is_finite() || !phase.is_finite() {
                continue;
            }
            let amplitude_scale = (baseline.amplitude_mad[index] * 1.4826)
                .max(baseline.amplitude_median[index].abs() * 0.015)
                .max(EPSILON);
            let amplitude_z = ((amplitude - baseline.amplitude_median[index]).abs()
                / amplitude_scale)
                .min(8.0)
                / 8.0;
            let phase_scale = (baseline.phase_mad[index] * 1.4826).max(0.08);
            let phase_z = (wrapped_phase_delta(phase, baseline.phase_center[index]) / phase_scale)
                .min(8.0)
                / 8.0;
            amplitude_total += amplitude_z;
            phase_total += phase_z;
            used += 1;
        }

        let amplitude_score = if used > 0 {
            amplitude_total / used as f32
        } else {
            0.0
        };
        let phase_score = if used > 0 {
            phase_total / used as f32
        } else {
            0.0
        };
        let rssi_scale = (baseline.rssi_mad * 1.4826).max(1.5);
        let rssi_score = (((frame.rssi as f32 - baseline.rssi_median).abs() / rssi_scale)
            .min(8.0)
            / 8.0)
            .clamp(0.0, 1.0);

        let robust_residual =
            (amplitude_score * 0.58 + phase_score * 0.27 + rssi_score * 0.15).clamp(0.0, 1.0);
        let combined = (robust_residual * 0.82 + motion_score.clamp(0.0, 1.0) * 0.18)
            .clamp(0.0, 1.0);

        // Drift is updated only when the transmitter is fixed and current motion is low.
        // This prevents a person from being absorbed into the empty-room model.
        if stationarity_score >= 0.85 && motion_score <= 0.12 {
            self.drift_index = self.drift_index * 0.995 + robust_residual * 0.005;
            self.drift_warning = self.drift_index >= self.config.drift_warning_threshold;
            self.recalibration_required =
                self.drift_index >= self.config.drift_recalibration_threshold;
        }

        if self.recalibration_required {
            // Keep the completed statistics for diagnostics, but block READY/solver use.
            self.invalidation_reason = Some(BaselineInvalidationReason::DriftExceeded);
            self.invalidated_at_ms.get_or_insert_with(now_ms);
        }

        LinkObservation {
            amplitude_score,
            phase_score,
            rssi_score,
            motion_score,
            combined,
            drift_index: self.drift_index,
            stable_carriers_used: used,
            baseline_ready: self.ready(),
        }
    }

    pub fn invalidate(&mut self, reason: BaselineInvalidationReason) {
        self.completed = None;
        self.amplitude_samples.clear();
        self.phase_samples.clear();
        self.rssi_samples.clear();
        self.accepted_frames = 0;
        self.total_values = 0;
        self.finite_values = 0;
        self.n_subcarriers = 0;
        self.channel = None;
        self.drift_index = 0.0;
        self.drift_warning = false;
        self.recalibration_required = false;
        self.invalidated_at_ms = Some(now_ms());
        self.invalidation_reason = Some(reason);
    }

    pub fn ready(&self) -> bool {
        self.completed
            .as_ref()
            .is_some_and(|baseline| baseline.quality >= self.config.minimum_quality)
            && !self.recalibration_required
    }

    pub fn quality(&self) -> f32 {
        self.completed.as_ref().map_or(0.0, |baseline| baseline.quality)
    }

    pub fn calibration_id(&self) -> Option<&str> {
        self.completed
            .as_ref()
            .map(|baseline| baseline.calibration_id.as_str())
    }

    pub fn channel(&self) -> Option<u8> {
        self.channel
    }

    pub fn summary(&self) -> BaselineSummary {
        let (stable, unstable, stable_ratio, finite_ratio, packet_quality, phase_stability, quality, created_at_ms, calibration_id) =
            if let Some(baseline) = &self.completed {
                let unstable = baseline
                    .stable_mask
                    .iter()
                    .enumerate()
                    .filter_map(|(index, stable)| (!*stable).then_some(index))
                    .collect::<Vec<_>>();
                let stable = baseline.stable_mask.len().saturating_sub(unstable.len());
                (
                    stable,
                    unstable,
                    stable as f32 / baseline.stable_mask.len().max(1) as f32,
                    baseline.finite_ratio,
                    baseline.packet_quality,
                    baseline.phase_stability,
                    baseline.quality,
                    Some(baseline.created_at_ms),
                    Some(baseline.calibration_id.clone()),
                )
            } else {
                (
                    0,
                    Vec::new(),
                    0.0,
                    if self.total_values == 0 {
                        0.0
                    } else {
                        self.finite_values as f32 / self.total_values as f32
                    },
                    self.packet_quality_ema,
                    0.0,
                    0.0,
                    None,
                    None,
                )
            };

        BaselineSummary {
            calibration_id,
            collecting: self.completed.is_none(),
            ready: self.ready(),
            frames_collected: self.accepted_frames,
            target_frames: self.config.target_frames,
            channel: self.channel,
            subcarrier_count: self.n_subcarriers,
            stable_subcarriers: stable,
            unstable_subcarriers: unstable,
            stable_carrier_ratio: stable_ratio,
            finite_sample_ratio: finite_ratio,
            packet_quality,
            phase_stability,
            quality,
            drift_index: self.drift_index,
            drift_warning: self.drift_warning,
            recalibration_required: self.recalibration_required,
            invalidated_at_ms: self.invalidated_at_ms,
            invalidation_reason: self.invalidation_reason.clone(),
            created_at_ms,
        }
    }

    fn prepare_carriers(&mut self, count: usize) {
        self.n_subcarriers = count;
        self.amplitude_samples = (0..count)
            .map(|_| Vec::with_capacity(self.config.target_frames))
            .collect();
        self.phase_samples = (0..count)
            .map(|_| Vec::with_capacity(self.config.target_frames))
            .collect();
    }

    fn collect_frame(&mut self, frame: &CsiFrame, stationarity_score: f32) {
        if stationarity_score < 0.80 || frame.amplitudes.len() != self.n_subcarriers {
            return;
        }
        for index in 0..self.n_subcarriers {
            self.total_values += 2;
            let amplitude = frame.amplitudes[index];
            let phase = frame.phases.get(index).copied().unwrap_or(f32::NAN);
            if amplitude.is_finite() {
                self.amplitude_samples[index].push(amplitude);
                self.finite_values += 1;
            }
            if phase.is_finite() {
                self.phase_samples[index].push(phase);
                self.finite_values += 1;
            }
        }
        self.rssi_samples.push(frame.rssi as f32);
        self.accepted_frames += 1;
    }

    fn finalize(&mut self) {
        if self.n_subcarriers == 0 || self.accepted_frames < self.config.target_frames {
            return;
        }
        let mut amplitude_median = Vec::with_capacity(self.n_subcarriers);
        let mut amplitude_mad = Vec::with_capacity(self.n_subcarriers);
        let mut phase_center = Vec::with_capacity(self.n_subcarriers);
        let mut phase_mad = Vec::with_capacity(self.n_subcarriers);
        let mut stable_mask = Vec::with_capacity(self.n_subcarriers);
        let mut phase_stability_total = 0.0f32;

        for index in 0..self.n_subcarriers {
            let amp_median = median(&self.amplitude_samples[index]).unwrap_or(0.0);
            let amp_mad = mad(&self.amplitude_samples[index], amp_median).unwrap_or(f32::INFINITY);
            let phase = circular_center(&self.phase_samples[index]).unwrap_or(0.0);
            let phase_deviation = self.phase_samples[index]
                .iter()
                .filter(|value| value.is_finite())
                .map(|value| wrapped_phase_delta(*value, phase))
                .collect::<Vec<_>>();
            let phase_median_deviation = median(&phase_deviation).unwrap_or(f32::INFINITY);
            let amp_ratio = amp_mad / amp_median.abs().max(1.0);
            let stable = amp_median > EPSILON
                && amp_ratio <= self.config.max_amplitude_mad_ratio
                && phase_median_deviation <= self.config.max_phase_mad_rad
                && self.amplitude_samples[index].len() >= self.config.target_frames * 9 / 10
                && self.phase_samples[index].len() >= self.config.target_frames * 9 / 10;
            amplitude_median.push(amp_median);
            amplitude_mad.push(amp_mad.max(EPSILON));
            phase_center.push(phase);
            phase_mad.push(phase_median_deviation.max(0.01));
            stable_mask.push(stable);
            if stable {
                phase_stability_total +=
                    (1.0 - phase_median_deviation / std::f32::consts::PI).clamp(0.0, 1.0);
            }
        }

        let stable_count = stable_mask.iter().filter(|stable| **stable).count();
        let stable_ratio = stable_count as f32 / self.n_subcarriers.max(1) as f32;
        let finite_ratio = if self.total_values == 0 {
            0.0
        } else {
            self.finite_values as f32 / self.total_values as f32
        };
        let phase_stability = if stable_count == 0 {
            0.0
        } else {
            phase_stability_total / stable_count as f32
        };
        let sample_score =
            (self.accepted_frames as f32 / self.config.target_frames as f32).clamp(0.0, 1.0);
        let quality = (sample_score
            * stable_ratio
            * finite_ratio
            * self.packet_quality_ema.clamp(0.0, 1.0)
            * phase_stability.max(0.10))
            .sqrt()
            .clamp(0.0, 1.0);
        let rssi_median = median(&self.rssi_samples).unwrap_or(-100.0);
        let rssi_mad = mad(&self.rssi_samples, rssi_median).unwrap_or(2.0).max(0.5);
        let created_at_ms = now_ms();
        let channel = self.channel.unwrap_or(0);
        let calibration_id = format!(
            "baseline-{}-{}-{}-{:03}",
            created_at_ms,
            channel,
            self.n_subcarriers,
            (quality * 1000.0).round() as u16
        );
        self.completed = Some(CompletedBaseline {
            calibration_id,
            channel,
            n_subcarriers: self.n_subcarriers,
            amplitude_median,
            amplitude_mad,
            phase_center,
            phase_mad,
            stable_mask,
            rssi_median,
            rssi_mad,
            quality,
            finite_ratio,
            packet_quality: self.packet_quality_ema,
            phase_stability,
            created_at_ms,
        });
        self.invalidated_at_ms = None;
        self.invalidation_reason = None;
    }
}

fn median(values: &[f32]) -> Option<f32> {
    let mut finite = values
        .iter()
        .copied()
        .filter(|value| value.is_finite())
        .collect::<Vec<_>>();
    if finite.is_empty() {
        return None;
    }
    finite.sort_by(|left, right| left.total_cmp(right));
    let middle = finite.len() / 2;
    if finite.len() % 2 == 0 {
        Some((finite[middle - 1] + finite[middle]) * 0.5)
    } else {
        Some(finite[middle])
    }
}

fn mad(values: &[f32], center: f32) -> Option<f32> {
    let deviations = values
        .iter()
        .copied()
        .filter(|value| value.is_finite())
        .map(|value| (value - center).abs())
        .collect::<Vec<_>>();
    median(&deviations)
}

fn circular_center(values: &[f32]) -> Option<f32> {
    let mut sin = 0.0f32;
    let mut cos = 0.0f32;
    let mut count = 0usize;
    for value in values.iter().copied().filter(|value| value.is_finite()) {
        sin += value.sin();
        cos += value.cos();
        count += 1;
    }
    (count > 0).then(|| sin.atan2(cos))
}

fn wrapped_phase_delta(left: f32, right: f32) -> f32 {
    let mut delta = (left - right).abs() % (2.0 * std::f32::consts::PI);
    if delta > std::f32::consts::PI {
        delta = 2.0 * std::f32::consts::PI - delta;
    }
    delta.abs()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::{build_v2_test_frame, parse_adr018};

    #[test]
    fn robust_baseline_rejects_unstable_carrier() {
        let mut engine = BaselineEngine::new(BaselineConfig {
            target_frames: 40,
            ..BaselineConfig::default()
        });
        for sequence in 0..40 {
            let mut frame = parse_adr018(&build_v2_test_frame(1, 1, sequence)).unwrap();
            if sequence % 2 == 0 {
                frame.amplitudes[0] *= 10.0;
            }
            engine.ingest(&frame, 1.0, 1.0, 0.0);
        }
        let summary = engine.summary();
        assert_eq!(summary.frames_collected, 40);
        assert!(summary.unstable_subcarriers.contains(&0));
    }

    #[test]
    fn channel_change_invalidates_baseline() {
        let mut engine = BaselineEngine::new(BaselineConfig {
            target_frames: 40,
            ..BaselineConfig::default()
        });
        for sequence in 0..40 {
            let frame = parse_adr018(&build_v2_test_frame(1, 1, sequence)).unwrap();
            engine.ingest(&frame, 1.0, 1.0, 0.0);
        }
        let mut changed = parse_adr018(&build_v2_test_frame(1, 1, 50)).unwrap();
        changed.channel = 11;
        engine.ingest(&changed, 1.0, 1.0, 0.0);
        assert_eq!(
            engine.summary().invalidation_reason,
            Some(BaselineInvalidationReason::ChannelChanged)
        );
        assert!(!engine.ready());
    }
}

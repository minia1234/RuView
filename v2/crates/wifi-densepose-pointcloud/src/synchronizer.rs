//! Per-link bounded packet reordering, duplicate/late classification and clock diagnostics.
//!
//! The synchronizer never allocates memory proportional to a sequence gap. Large jumps are
//! summarized as bounded ranges and reset the reorder window instead of expanding buffers.

use crate::parser::CsiFrame;
use serde::Serialize;
use std::collections::{BTreeMap, VecDeque};

const DEFAULT_REORDER_WINDOW: usize = 24;
const DEFAULT_MAX_GAP: u32 = 4_096;
const MAX_GAP_RANGES: usize = 64;
const RECENT_SEQUENCE_CACHE: usize = 128;

#[derive(Clone, Debug, Serialize)]
pub struct MissingRange {
    pub start: u32,
    pub end: u32,
    pub count: u32,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct SequenceDiagnostics {
    pub received: u64,
    pub delivered: u64,
    pub duplicates: u64,
    pub late_packets: u64,
    pub reordered_packets: u64,
    pub missing_packets: u64,
    pub discontinuities: u64,
    pub buffered_packets: usize,
    pub expected_sequence: Option<u32>,
    pub missing_ranges: Vec<MissingRange>,
    pub clock_offset_us: Option<f64>,
    pub clock_drift_ppm: Option<f64>,
    pub timestamp_match_rate: f32,
}

#[derive(Clone, Debug)]
pub struct SynchronizedFrame {
    pub frame: CsiFrame,
    pub reordered: bool,
    pub missing_before: u32,
}

#[derive(Clone, Debug, Default)]
struct ClockEstimator {
    samples: u64,
    matched: u64,
    offset_ema_us: Option<f64>,
    previous_remote_us: Option<u64>,
    previous_arrival_us: Option<u64>,
    drift_ema_ppm: Option<f64>,
}

impl ClockEstimator {
    fn observe(&mut self, remote_us: u64, arrival_ms: u64) {
        self.samples += 1;
        if remote_us == 0 {
            return;
        }
        self.matched += 1;
        let arrival_us = arrival_ms.saturating_mul(1_000);
        let offset = arrival_us as f64 - remote_us as f64;
        self.offset_ema_us = Some(match self.offset_ema_us {
            Some(previous) => previous * 0.98 + offset * 0.02,
            None => offset,
        });

        if let (Some(previous_remote), Some(previous_arrival)) =
            (self.previous_remote_us, self.previous_arrival_us)
        {
            let remote_delta = remote_us.wrapping_sub(previous_remote) as f64;
            let arrival_delta = arrival_us.saturating_sub(previous_arrival) as f64;
            if remote_delta >= 1_000.0 && arrival_delta > 0.0 {
                let ppm = ((arrival_delta / remote_delta) - 1.0) * 1_000_000.0;
                if ppm.is_finite() && ppm.abs() <= 100_000.0 {
                    self.drift_ema_ppm = Some(match self.drift_ema_ppm {
                        Some(previous) => previous * 0.95 + ppm * 0.05,
                        None => ppm,
                    });
                }
            }
        }
        self.previous_remote_us = Some(remote_us);
        self.previous_arrival_us = Some(arrival_us);
    }

    fn match_rate(&self) -> f32 {
        if self.samples == 0 {
            0.0
        } else {
            self.matched as f32 / self.samples as f32
        }
    }
}

pub struct LinkSynchronizer {
    reorder_window: usize,
    max_gap: u32,
    expected: Option<u32>,
    buffer: BTreeMap<u32, (CsiFrame, bool)>,
    recent_delivered: VecDeque<u32>,
    pending_missing: u32,
    diagnostics: SequenceDiagnostics,
    clock: ClockEstimator,
}

impl Default for LinkSynchronizer {
    fn default() -> Self {
        Self::new(DEFAULT_REORDER_WINDOW, DEFAULT_MAX_GAP)
    }
}

impl LinkSynchronizer {
    pub fn new(reorder_window: usize, max_gap: u32) -> Self {
        Self {
            reorder_window: reorder_window.clamp(2, 256),
            max_gap: max_gap.clamp(32, 1_000_000),
            expected: None,
            buffer: BTreeMap::new(),
            recent_delivered: VecDeque::with_capacity(RECENT_SEQUENCE_CACHE),
            pending_missing: 0,
            diagnostics: SequenceDiagnostics::default(),
            clock: ClockEstimator::default(),
        }
    }

    pub fn push(&mut self, frame: CsiFrame) -> Vec<SynchronizedFrame> {
        self.diagnostics.received += 1;
        self.clock.observe(frame.timestamp_us, frame.received_at_ms);

        let sequence = frame.sequence;
        if self.expected.is_none() {
            self.expected = Some(sequence);
        }
        let expected = self.expected.unwrap_or(sequence);
        let delta = signed_sequence_delta(sequence, expected);

        if delta < 0 {
            if self.recent_delivered.contains(&sequence) || self.buffer.contains_key(&sequence) {
                self.diagnostics.duplicates += 1;
            } else {
                self.diagnostics.late_packets += 1;
            }
            self.refresh_diagnostics();
            return Vec::new();
        }

        if self.buffer.contains_key(&sequence) {
            self.diagnostics.duplicates += 1;
            self.refresh_diagnostics();
            return Vec::new();
        }

        if delta as u64 > self.max_gap as u64 {
            self.record_gap(expected, delta as u32);
            self.buffer.clear();
            self.expected = Some(sequence);
            self.diagnostics.discontinuities += 1;
        } else if delta > 0 {
            self.diagnostics.reordered_packets += 1;
        }

        let expected = self.expected.unwrap_or(sequence);
        let delta = signed_sequence_delta(sequence, expected).max(0) as usize;
        if delta >= self.reorder_window {
            let advance = delta - self.reorder_window + 1;
            self.record_gap(expected, advance as u32);
            self.expected = Some(expected.wrapping_add(advance as u32));
        }

        let reordered = sequence != self.expected.unwrap_or(sequence);
        self.buffer.insert(sequence, (frame, reordered));
        let output = self.drain_contiguous();
        self.refresh_diagnostics();
        output
    }

    pub fn flush_bounded(&mut self) -> Vec<SynchronizedFrame> {
        if self.buffer.is_empty() {
            return Vec::new();
        }
        let expected = self.expected.unwrap_or_else(|| *self.buffer.keys().next().unwrap());
        let next = *self.buffer.keys().next().unwrap();
        let delta = signed_sequence_delta(next, expected).max(0) as u32;
        if delta > 0 {
            self.record_gap(expected, delta);
            self.expected = Some(next);
        }
        let output = self.drain_contiguous();
        self.refresh_diagnostics();
        output
    }

    pub fn diagnostics(&self) -> SequenceDiagnostics {
        let mut diagnostics = self.diagnostics.clone();
        diagnostics.buffered_packets = self.buffer.len();
        diagnostics.expected_sequence = self.expected;
        diagnostics.clock_offset_us = self.clock.offset_ema_us;
        diagnostics.clock_drift_ppm = self.clock.drift_ema_ppm;
        diagnostics.timestamp_match_rate = self.clock.match_rate();
        diagnostics
    }

    pub fn reset(&mut self) {
        self.expected = None;
        self.buffer.clear();
        self.recent_delivered.clear();
        self.pending_missing = 0;
        self.diagnostics = SequenceDiagnostics::default();
        self.clock = ClockEstimator::default();
    }

    fn drain_contiguous(&mut self) -> Vec<SynchronizedFrame> {
        let mut output = Vec::new();
        loop {
            let expected = match self.expected {
                Some(value) => value,
                None => break,
            };
            let Some((frame, reordered)) = self.buffer.remove(&expected) else {
                break;
            };
            let missing_before = std::mem::take(&mut self.pending_missing);
            output.push(SynchronizedFrame {
                frame,
                reordered,
                missing_before,
            });
            self.diagnostics.delivered += 1;
            self.recent_delivered.push_back(expected);
            while self.recent_delivered.len() > RECENT_SEQUENCE_CACHE {
                self.recent_delivered.pop_front();
            }
            self.expected = Some(expected.wrapping_add(1));
        }
        output
    }

    fn record_gap(&mut self, start: u32, count: u32) {
        if count == 0 {
            return;
        }
        self.diagnostics.missing_packets = self
            .diagnostics
            .missing_packets
            .saturating_add(count as u64);
        self.pending_missing = self.pending_missing.saturating_add(count);
        let range = MissingRange {
            start,
            end: start.wrapping_add(count.saturating_sub(1)),
            count,
        };
        self.diagnostics.missing_ranges.push(range);
        if self.diagnostics.missing_ranges.len() > MAX_GAP_RANGES {
            self.diagnostics.missing_ranges.remove(0);
        }
    }

    fn refresh_diagnostics(&mut self) {
        self.diagnostics.buffered_packets = self.buffer.len();
        self.diagnostics.expected_sequence = self.expected;
        self.diagnostics.clock_offset_us = self.clock.offset_ema_us;
        self.diagnostics.clock_drift_ppm = self.clock.drift_ema_ppm;
        self.diagnostics.timestamp_match_rate = self.clock.match_rate();
    }
}

fn signed_sequence_delta(sequence: u32, expected: u32) -> i64 {
    sequence.wrapping_sub(expected) as i32 as i64
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::{build_v2_test_frame, parse_adr018};

    fn frame(sequence: u32) -> CsiFrame {
        parse_adr018(&build_v2_test_frame(1, 1, sequence)).unwrap()
    }

    #[test]
    fn reorders_within_window() {
        let mut sync = LinkSynchronizer::new(8, 100);
        assert_eq!(sync.push(frame(10)).len(), 1);
        assert!(sync.push(frame(12)).is_empty());
        let output = sync.push(frame(11));
        assert_eq!(output.len(), 2);
        assert_eq!(output[0].frame.sequence, 11);
        assert_eq!(output[1].frame.sequence, 12);
        assert_eq!(sync.diagnostics().reordered_packets, 1);
    }

    #[test]
    fn classifies_duplicate_and_late() {
        let mut sync = LinkSynchronizer::default();
        sync.push(frame(1));
        sync.push(frame(1));
        sync.push(frame(0));
        let diagnostics = sync.diagnostics();
        assert_eq!(diagnostics.duplicates, 1);
        assert_eq!(diagnostics.late_packets, 1);
    }

    #[test]
    fn large_jump_is_bounded() {
        let mut sync = LinkSynchronizer::new(8, 64);
        sync.push(frame(1));
        let output = sync.push(frame(1_000_000));
        assert_eq!(output.len(), 1);
        let diagnostics = sync.diagnostics();
        assert_eq!(diagnostics.discontinuities, 1);
        assert!(diagnostics.missing_ranges.len() <= MAX_GAP_RANGES);
        assert!(diagnostics.buffered_packets <= 8);
    }
}

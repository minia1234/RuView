//! Hardware validation harness for FormMap T-01 through T-08.
//!
//! This module does not claim RF accuracy without measurements. It evaluates recorded
//! occupancy/system JSONL against optional ground-truth labels and marks missing evidence
//! as `not_run`, never as a pass.

use anyhow::{anyhow, Context, Result};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::fs::{self, File};
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

const OCCUPANCY_FILE: &str = "occupancy.jsonl";
const EVENTS_FILE: &str = "events.jsonl";
const DEFAULT_LABELS_FILE: &str = "validation-labels.jsonl";

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum TestId {
    T01,
    T02,
    T03,
    T04,
    T05,
    T06,
    T07,
    T08,
}

impl TestId {
    fn title(self) -> &'static str {
        match self {
            Self::T01 => "empty-room false presence",
            Self::T02 => "center stationary presence",
            Self::T03 => "nine-position centroid error",
            Self::T04 => "walking direction agreement",
            Self::T05 => "standing/sitting/lying height state",
            Self::T06 => "single-link dropout degraded continuity",
            Self::T07 => "station movement exclusion latency",
            Self::T08 => "door/furniture environmental drift warning",
        }
    }

    fn target(self) -> &'static str {
        match self {
            Self::T01 => "false presence <= 5%",
            Self::T02 => "presence detection >= 90%",
            Self::T03 => "mean centroid error <= 0.8 m",
            Self::T04 => "direction agreement >= 80%",
            Self::T05 => "height-state accuracy >= 75%",
            Self::T06 => "DEGRADED output continues after one link drops",
            Self::T07 => "movement detected and link excluded within 5 s",
            Self::T08 => "drift warning emitted after environment change",
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
pub struct ValidationLabel {
    pub timestamp_ms: u64,
    #[serde(default)]
    pub expected_present: Option<bool>,
    #[serde(default)]
    pub position_m: Option<[f32; 3]>,
    #[serde(default)]
    pub state: Option<String>,
    #[serde(default)]
    pub event: Option<String>,
    #[serde(default)]
    pub link_id: Option<u32>,
}

#[derive(Clone, Debug)]
struct OccupancySample {
    timestamp_ms: u64,
    cells: usize,
    confidence: f32,
    centroid_m: Option<[f32; 3]>,
    height_state: Option<String>,
    active_links: usize,
    source: String,
}

#[derive(Clone, Debug)]
struct SystemSample {
    timestamp_ms: u64,
    state: String,
    reason_code: String,
    active_links: usize,
    drift_index: f32,
    recalibration_required: bool,
}

#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TestStatus {
    Pass,
    Fail,
    NotRun,
}

#[derive(Clone, Debug, Serialize)]
pub struct TestResult {
    pub id: TestId,
    pub title: String,
    pub target: String,
    pub status: TestStatus,
    pub metric_name: String,
    pub metric_value: Option<f64>,
    pub sample_count: usize,
    pub details: String,
}

#[derive(Clone, Debug, Serialize)]
pub struct ValidationReport {
    pub schema: String,
    pub generated_at_ms: u64,
    pub capture_id: String,
    pub capture_directory: String,
    pub labels_file: Option<String>,
    pub measured_source_only: bool,
    pub passed: usize,
    pub failed: usize,
    pub not_run: usize,
    pub tests: Vec<TestResult>,
}

pub fn run_hardware_validation(
    capture_root: impl AsRef<Path>,
    capture_id: &str,
    labels_path: Option<&Path>,
    output_path: Option<&Path>,
) -> Result<ValidationReport> {
    validate_capture_id(capture_id)?;
    let directory = capture_root.as_ref().join(capture_id);
    if !directory.is_dir() {
        return Err(anyhow!("capture not found: {}", directory.display()));
    }
    let occupancy = read_occupancy(&directory.join(OCCUPANCY_FILE))?;
    let systems = read_system(&directory.join(EVENTS_FILE))?;
    let default_labels = directory.join(DEFAULT_LABELS_FILE);
    let resolved_labels = labels_path
        .map(PathBuf::from)
        .or_else(|| default_labels.is_file().then_some(default_labels));
    let labels = match &resolved_labels {
        Some(path) => read_json_lines::<ValidationLabel>(path)?,
        None => Vec::new(),
    };
    let measured_source_only = occupancy
        .iter()
        .all(|sample| sample.source == "measured" || sample.source == "unavailable");

    let tests = vec![
        evaluate_t01(&occupancy, &labels),
        evaluate_t02(&occupancy, &labels),
        evaluate_t03(&occupancy, &labels),
        evaluate_t04(&occupancy, &labels),
        evaluate_t05(&occupancy, &labels),
        evaluate_t06(&occupancy, &systems, &labels),
        evaluate_t07(&systems, &labels),
        evaluate_t08(&systems, &labels),
    ];
    let passed = tests
        .iter()
        .filter(|test| matches!(test.status, TestStatus::Pass))
        .count();
    let failed = tests
        .iter()
        .filter(|test| matches!(test.status, TestStatus::Fail))
        .count();
    let not_run = tests
        .iter()
        .filter(|test| matches!(test.status, TestStatus::NotRun))
        .count();
    let report = ValidationReport {
        schema: "formmap-hardware-validation/v1".to_string(),
        generated_at_ms: now_ms(),
        capture_id: capture_id.to_string(),
        capture_directory: directory.to_string_lossy().into_owned(),
        labels_file: resolved_labels.map(|path| path.to_string_lossy().into_owned()),
        measured_source_only,
        passed,
        failed,
        not_run,
        tests,
    };
    let output = output_path
        .map(PathBuf::from)
        .unwrap_or_else(|| directory.join("hardware-validation-report.json"));
    fs::write(&output, serde_json::to_string_pretty(&report)?)
        .with_context(|| format!("failed to write {}", output.display()))?;
    Ok(report)
}

pub fn suite_definition() -> Value {
    serde_json::json!({
        "schema": "formmap-hardware-suite/v1",
        "tests": [
            {"id":"T-01","condition":"empty room for 30 minutes","target":"false presence <= 5%"},
            {"id":"T-02","condition":"stationary center position, 10 trials","target":"presence >= 90%"},
            {"id":"T-03","condition":"nine labelled room positions","target":"centroid error <= 0.8 m"},
            {"id":"T-04","condition":"labelled walking path","target":"direction agreement >= 80%"},
            {"id":"T-05","condition":"standing, sitting and lying labels","target":"height state >= 75%"},
            {"id":"T-06","condition":"one active link removed","target":"DEGRADED output continues"},
            {"id":"T-07","condition":"fixed station is moved","target":"detect and exclude within 5 s"},
            {"id":"T-08","condition":"door or furniture changes","target":"drift warning emitted"}
        ],
        "honesty": "A test without recorded evidence is not_run, never pass."
    })
}

fn evaluate_t01(samples: &[OccupancySample], labels: &[ValidationLabel]) -> TestResult {
    let empty_ranges = labels
        .iter()
        .filter(|label| label.expected_present == Some(false))
        .collect::<Vec<_>>();
    if empty_ranges.is_empty() {
        return not_run(TestId::T01, "no expected_present=false labels");
    }
    let selected = samples
        .iter()
        .filter(|sample| nearest_label(sample.timestamp_ms, &empty_ranges).is_some())
        .collect::<Vec<_>>();
    if selected.is_empty() {
        return not_run(TestId::T01, "no occupancy samples overlap empty-room labels");
    }
    let false_positive = selected
        .iter()
        .filter(|sample| sample.cells > 0 && sample.confidence >= 0.35)
        .count();
    let rate = false_positive as f64 / selected.len() as f64;
    result(
        TestId::T01,
        rate <= 0.05,
        "false_presence_rate",
        rate,
        selected.len(),
        format!("{false_positive} false-positive frames"),
    )
}

fn evaluate_t02(samples: &[OccupancySample], labels: &[ValidationLabel]) -> TestResult {
    let present = labels
        .iter()
        .filter(|label| label.expected_present == Some(true))
        .collect::<Vec<_>>();
    if present.is_empty() {
        return not_run(TestId::T02, "no expected_present=true labels");
    }
    let selected = samples
        .iter()
        .filter(|sample| nearest_label(sample.timestamp_ms, &present).is_some())
        .collect::<Vec<_>>();
    if selected.is_empty() {
        return not_run(TestId::T02, "no occupancy samples overlap presence labels");
    }
    let detected = selected
        .iter()
        .filter(|sample| sample.cells > 0 && sample.confidence >= 0.35)
        .count();
    let rate = detected as f64 / selected.len() as f64;
    result(
        TestId::T02,
        rate >= 0.90,
        "presence_detection_rate",
        rate,
        selected.len(),
        format!("{detected} detected frames"),
    )
}

fn evaluate_t03(samples: &[OccupancySample], labels: &[ValidationLabel]) -> TestResult {
    let positioned = labels
        .iter()
        .filter(|label| label.position_m.is_some())
        .collect::<Vec<_>>();
    if positioned.len() < 3 {
        return not_run(TestId::T03, "at least three position labels are required");
    }
    let mut errors = Vec::new();
    for sample in samples {
        let Some(centroid) = sample.centroid_m else {
            continue;
        };
        let Some(label) = nearest_label(sample.timestamp_ms, &positioned) else {
            continue;
        };
        if let Some(expected) = label.position_m {
            errors.push(distance(centroid, expected) as f64);
        }
    }
    if errors.is_empty() {
        return not_run(TestId::T03, "no centroid samples overlap position labels");
    }
    let mean = errors.iter().sum::<f64>() / errors.len() as f64;
    result(
        TestId::T03,
        mean <= 0.8,
        "mean_centroid_error_m",
        mean,
        errors.len(),
        format!("P95 {:.3} m", percentile(&mut errors.clone(), 0.95)),
    )
}

fn evaluate_t04(samples: &[OccupancySample], labels: &[ValidationLabel]) -> TestResult {
    let positioned = labels
        .iter()
        .filter(|label| label.position_m.is_some())
        .collect::<Vec<_>>();
    if positioned.len() < 2 {
        return not_run(TestId::T04, "walking path position labels are missing");
    }
    let mut compared = 0usize;
    let mut agreed = 0usize;
    for pair in samples.windows(2) {
        let (Some(left), Some(right)) = (pair[0].centroid_m, pair[1].centroid_m) else {
            continue;
        };
        let (Some(label_left), Some(label_right)) = (
            nearest_label(pair[0].timestamp_ms, &positioned),
            nearest_label(pair[1].timestamp_ms, &positioned),
        ) else {
            continue;
        };
        let (Some(expected_left), Some(expected_right)) =
            (label_left.position_m, label_right.position_m)
        else {
            continue;
        };
        let measured = [right[0] - left[0], right[2] - left[2]];
        let expected = [
            expected_right[0] - expected_left[0],
            expected_right[2] - expected_left[2],
        ];
        let measured_norm = (measured[0] * measured[0] + measured[1] * measured[1]).sqrt();
        let expected_norm = (expected[0] * expected[0] + expected[1] * expected[1]).sqrt();
        if measured_norm < 0.02 || expected_norm < 0.02 {
            continue;
        }
        let cosine = (measured[0] * expected[0] + measured[1] * expected[1])
            / (measured_norm * expected_norm);
        compared += 1;
        if cosine >= 0.0 {
            agreed += 1;
        }
    }
    if compared == 0 {
        return not_run(TestId::T04, "no moving sample pairs could be compared");
    }
    let rate = agreed as f64 / compared as f64;
    result(
        TestId::T04,
        rate >= 0.80,
        "direction_agreement_rate",
        rate,
        compared,
        format!("{agreed} direction matches"),
    )
}

fn evaluate_t05(samples: &[OccupancySample], labels: &[ValidationLabel]) -> TestResult {
    let states = labels
        .iter()
        .filter(|label| label.state.is_some())
        .collect::<Vec<_>>();
    if states.is_empty() {
        return not_run(TestId::T05, "standing/sitting/lying labels are missing");
    }
    let mut compared = 0usize;
    let mut correct = 0usize;
    for sample in samples {
        let Some(measured) = &sample.height_state else {
            continue;
        };
        let Some(label) = nearest_label(sample.timestamp_ms, &states) else {
            continue;
        };
        let Some(expected) = &label.state else {
            continue;
        };
        compared += 1;
        if normalize_state(measured) == normalize_state(expected) {
            correct += 1;
        }
    }
    if compared == 0 {
        return not_run(TestId::T05, "no height-state output overlaps labels");
    }
    let accuracy = correct as f64 / compared as f64;
    result(
        TestId::T05,
        accuracy >= 0.75,
        "height_state_accuracy",
        accuracy,
        compared,
        format!("{correct} correct state frames"),
    )
}

fn evaluate_t06(
    occupancy: &[OccupancySample],
    systems: &[SystemSample],
    labels: &[ValidationLabel],
) -> TestResult {
    let Some(drop_label) = labels
        .iter()
        .find(|label| label.event.as_deref() == Some("link_dropout"))
    else {
        return not_run(TestId::T06, "link_dropout event label is missing");
    };
    let degraded = systems.iter().any(|sample| {
        sample.timestamp_ms >= drop_label.timestamp_ms
            && sample.state == "DEGRADED"
            && sample.reason_code.contains("LINK")
    });
    let continued = occupancy.iter().any(|sample| {
        sample.timestamp_ms >= drop_label.timestamp_ms
            && sample.active_links > 0
            && sample.timestamp_ms <= drop_label.timestamp_ms.saturating_add(10_000)
    });
    result(
        TestId::T06,
        degraded && continued,
        "degraded_continuity",
        if degraded && continued { 1.0 } else { 0.0 },
        systems.len(),
        format!("degraded={degraded}, output_continued={continued}"),
    )
}

fn evaluate_t07(systems: &[SystemSample], labels: &[ValidationLabel]) -> TestResult {
    let Some(label) = labels
        .iter()
        .find(|label| label.event.as_deref() == Some("station_moved"))
    else {
        return not_run(TestId::T07, "station_moved event label is missing");
    };
    let detected = systems.iter().find(|sample| {
        sample.timestamp_ms >= label.timestamp_ms
            && sample.timestamp_ms <= label.timestamp_ms.saturating_add(5_000)
            && (sample.reason_code.contains("STATION")
                || sample.reason_code.contains("BASELINE")
                || sample.state == "CALIBRATING")
    });
    let latency = detected.map(|sample| sample.timestamp_ms.saturating_sub(label.timestamp_ms));
    TestResult {
        id: TestId::T07,
        title: TestId::T07.title().to_string(),
        target: TestId::T07.target().to_string(),
        status: if latency.is_some() {
            TestStatus::Pass
        } else {
            TestStatus::Fail
        },
        metric_name: "movement_detection_latency_ms".to_string(),
        metric_value: latency.map(|value| value as f64),
        sample_count: systems.len(),
        details: format!("link_id={:?}", label.link_id),
    }
}

fn evaluate_t08(systems: &[SystemSample], labels: &[ValidationLabel]) -> TestResult {
    let Some(label) = labels.iter().find(|label| {
        matches!(
            label.event.as_deref(),
            Some("door_changed") | Some("furniture_changed")
        )
    }) else {
        return not_run(TestId::T08, "door_changed/furniture_changed label is missing");
    };
    let warning = systems.iter().find(|sample| {
        sample.timestamp_ms >= label.timestamp_ms
            && (sample.reason_code == "ENVIRONMENT_DRIFT"
                || sample.recalibration_required
                || sample.drift_index >= 0.35)
    });
    TestResult {
        id: TestId::T08,
        title: TestId::T08.title().to_string(),
        target: TestId::T08.target().to_string(),
        status: if warning.is_some() {
            TestStatus::Pass
        } else {
            TestStatus::Fail
        },
        metric_name: "drift_warning_emitted".to_string(),
        metric_value: warning.map(|sample| sample.drift_index as f64),
        sample_count: systems.len(),
        details: warning
            .map(|sample| format!("reason={}", sample.reason_code))
            .unwrap_or_else(|| "no drift warning after labelled change".to_string()),
    }
}

fn result(
    id: TestId,
    passed: bool,
    metric_name: &str,
    metric_value: f64,
    sample_count: usize,
    details: String,
) -> TestResult {
    TestResult {
        id,
        title: id.title().to_string(),
        target: id.target().to_string(),
        status: if passed {
            TestStatus::Pass
        } else {
            TestStatus::Fail
        },
        metric_name: metric_name.to_string(),
        metric_value: Some(metric_value),
        sample_count,
        details,
    }
}

fn not_run(id: TestId, reason: &str) -> TestResult {
    TestResult {
        id,
        title: id.title().to_string(),
        target: id.target().to_string(),
        status: TestStatus::NotRun,
        metric_name: "missing_evidence".to_string(),
        metric_value: None,
        sample_count: 0,
        details: reason.to_string(),
    }
}

fn nearest_label<'a>(timestamp_ms: u64, labels: &[&'a ValidationLabel]) -> Option<&'a ValidationLabel> {
    labels
        .iter()
        .copied()
        .filter_map(|label| {
            let delta = timestamp_ms.abs_diff(label.timestamp_ms);
            (delta <= 1_000).then_some((delta, label))
        })
        .min_by_key(|item| item.0)
        .map(|(_, label)| label)
}

fn read_occupancy(path: &Path) -> Result<Vec<OccupancySample>> {
    let values = read_json_lines::<Value>(path)?;
    Ok(values
        .into_iter()
        .filter_map(|value| {
            let timestamp_ms = value
                .get("timestamp")
                .or_else(|| value.get("timestamp_ms"))?
                .as_u64()?;
            let cells = value.get("cells").and_then(Value::as_array).map_or(0, Vec::len);
            let confidence = value.get("confidence").and_then(Value::as_f64).unwrap_or(0.0) as f32;
            let centroid_m = parse_position(value.get("centroid_m"));
            let height_state = value
                .pointer("/height_layers/dominant")
                .and_then(Value::as_str)
                .map(ToString::to_string);
            let active_links = value
                .get("active_links")
                .and_then(Value::as_u64)
                .unwrap_or(0) as usize;
            let source = value
                .get("source")
                .and_then(Value::as_str)
                .unwrap_or("unavailable")
                .to_string();
            Some(OccupancySample {
                timestamp_ms,
                cells,
                confidence,
                centroid_m,
                height_state,
                active_links,
                source,
            })
        })
        .collect())
}

fn read_system(path: &Path) -> Result<Vec<SystemSample>> {
    let values = read_json_lines::<Value>(path)?;
    Ok(values
        .into_iter()
        .filter_map(|value| {
            let timestamp_ms = value
                .get("timestamp_ms")
                .or_else(|| value.get("timestamp"))?
                .as_u64()?;
            Some(SystemSample {
                timestamp_ms,
                state: value
                    .get("state")
                    .and_then(Value::as_str)
                    .unwrap_or("OFFLINE")
                    .to_string(),
                reason_code: value
                    .get("reason_code")
                    .and_then(Value::as_str)
                    .unwrap_or("UNKNOWN")
                    .to_string(),
                active_links: value
                    .get("active_links")
                    .and_then(Value::as_u64)
                    .unwrap_or(0) as usize,
                drift_index: value
                    .get("drift_index")
                    .and_then(Value::as_f64)
                    .unwrap_or(0.0) as f32,
                recalibration_required: value
                    .get("recalibration_required")
                    .and_then(Value::as_bool)
                    .unwrap_or(false),
            })
        })
        .collect())
}

fn read_json_lines<T: for<'de> Deserialize<'de>>(path: &Path) -> Result<Vec<T>> {
    if !path.is_file() {
        return Ok(Vec::new());
    }
    let file = File::open(path).with_context(|| format!("failed to open {}", path.display()))?;
    let reader = BufReader::new(file);
    let mut output = Vec::new();
    for (line_number, line) in reader.lines().enumerate() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        output.push(serde_json::from_str(&line).with_context(|| {
            format!("invalid JSONL at {}:{}", path.display(), line_number + 1)
        })?);
    }
    Ok(output)
}

fn parse_position(value: Option<&Value>) -> Option<[f32; 3]> {
    let values = value?.as_array()?;
    if values.len() != 3 {
        return None;
    }
    Some([
        values[0].as_f64()? as f32,
        values[1].as_f64()? as f32,
        values[2].as_f64()? as f32,
    ])
}

fn normalize_state(value: &str) -> &'static str {
    match value.to_ascii_lowercase().as_str() {
        "high" | "standing" | "stand" => "standing",
        "mid" | "sitting" | "sit" => "sitting",
        "low" | "lying" | "lie" => "lying",
        _ => "unknown",
    }
}

fn percentile(values: &mut [f64], percentile: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    values.sort_by(|left, right| left.total_cmp(right));
    let index = ((values.len() - 1) as f64 * percentile.clamp(0.0, 1.0)).round() as usize;
    values[index]
}

fn distance(left: [f32; 3], right: [f32; 3]) -> f32 {
    let dx = left[0] - right[0];
    let dy = left[1] - right[1];
    let dz = left[2] - right[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn validate_capture_id(value: &str) -> Result<()> {
    if value.is_empty()
        || value.len() > 128
        || !value
            .chars()
            .all(|character| character.is_ascii_alphanumeric() || matches!(character, '-' | '_'))
    {
        return Err(anyhow!("invalid capture id"));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn suite_contains_all_hardware_tests() {
        let suite = suite_definition();
        assert_eq!(suite["tests"].as_array().unwrap().len(), 8);
    }

    #[test]
    fn missing_evidence_never_passes() {
        let result = evaluate_t01(&[], &[]);
        assert!(matches!(result.status, TestStatus::NotRun));
    }
}

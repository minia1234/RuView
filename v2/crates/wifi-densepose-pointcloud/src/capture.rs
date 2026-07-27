//! Raw ADR-018 capture proxy, provenance manifest and deterministic replay support.
//!
//! ESP32 nodes send CSI to the public capture socket. Every datagram is optionally
//! persisted verbatim and then forwarded to the private FormMap ingest socket.
//! Occupancy and system-event JSONL are recorded beside the raw stream so T-01..T-08
//! can be evaluated without inventing evidence.

use anyhow::{anyhow, Context, Result};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::fs::{self, File};
use std::io::{BufReader, BufWriter, ErrorKind, Read, Write};
use std::net::UdpSocket;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

pub type SharedCapture = Arc<Mutex<CaptureManager>>;

const CAPTURE_SCHEMA: &str = "formmap-capture/v2";
const RAW_FILE_NAME: &str = "raw-csi.bin";
const OCCUPANCY_FILE_NAME: &str = "occupancy.jsonl";
const EVENTS_FILE_NAME: &str = "events.jsonl";
const LAYOUT_FILE_NAME: &str = "layout.json";
const PROVENANCE_FILE_NAME: &str = "provenance.json";
const MANIFEST_FILE_NAME: &str = "manifest.json";
const MAX_DATAGRAM_BYTES: usize = 65_535;

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

fn now_us() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_micros() as u64
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CaptureProvenance {
    pub layout: Value,
    pub firmware: Vec<Value>,
    pub code_version: String,
    pub code_commit: String,
    pub protocol_versions: Vec<u8>,
    pub calibration_id: String,
    pub solver: Value,
    pub source_honesty: Value,
}

impl Default for CaptureProvenance {
    fn default() -> Self {
        Self {
            layout: Value::Null,
            firmware: Vec::new(),
            code_version: env!("CARGO_PKG_VERSION").to_string(),
            code_commit: option_env!("GIT_COMMIT_SHA")
                .unwrap_or("unknown")
                .to_string(),
            protocol_versions: Vec::new(),
            calibration_id: "unavailable".to_string(),
            solver: Value::Null,
            source_honesty: serde_json::json!({
                "camera": false,
                "skeleton": false,
                "automatic_simulation": false
            }),
        }
    }
}

impl CaptureProvenance {
    pub fn from_value(value: Value) -> Self {
        let layout = value.get("layout").cloned().unwrap_or(Value::Null);
        let firmware = value
            .get("firmware_versions")
            .and_then(Value::as_array)
            .cloned()
            .unwrap_or_default();
        let code_version = value
            .get("code_version")
            .and_then(Value::as_str)
            .unwrap_or(env!("CARGO_PKG_VERSION"))
            .to_string();
        let code_commit = value
            .get("code_commit")
            .and_then(Value::as_str)
            .unwrap_or("unknown")
            .to_string();
        let protocol_versions = value
            .get("protocol_versions")
            .and_then(Value::as_array)
            .map(|values| {
                values
                    .iter()
                    .filter_map(Value::as_u64)
                    .filter_map(|value| u8::try_from(value).ok())
                    .collect::<Vec<_>>()
            })
            .unwrap_or_default();
        let calibration_id = value
            .get("calibration_id")
            .and_then(Value::as_str)
            .unwrap_or("unavailable")
            .to_string();
        let solver = value.get("solver").cloned().unwrap_or(Value::Null);
        let source_honesty = value
            .get("source_honesty")
            .cloned()
            .unwrap_or(Value::Null);
        Self {
            layout,
            firmware,
            code_version,
            code_commit,
            protocol_versions,
            calibration_id,
            solver,
            source_honesty,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CaptureManifest {
    pub schema: String,
    pub session_id: String,
    pub label: Option<String>,
    pub started_at_ms: u64,
    pub ended_at_ms: Option<u64>,
    pub source: String,
    pub packet_count: u64,
    pub byte_count: u64,
    #[serde(default)]
    pub occupancy_frame_count: u64,
    #[serde(default)]
    pub event_count: u64,
    pub input_bind: String,
    pub forward_target: String,
    pub code_version: String,
    #[serde(default)]
    pub code_commit: String,
    pub protocol: String,
    #[serde(default)]
    pub protocol_versions: Vec<u8>,
    #[serde(default)]
    pub calibration_id: String,
    #[serde(default)]
    pub firmware: Vec<Value>,
    #[serde(default)]
    pub solver: Value,
    #[serde(default)]
    pub consent: bool,
    #[serde(default)]
    pub files: Vec<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ReplayStatus {
    pub capture_id: String,
    pub active: bool,
    pub speed: f32,
    pub packet_count: usize,
    pub packets_sent: usize,
    pub started_at_ms: u64,
    pub completed_at_ms: Option<u64>,
    pub error: Option<String>,
}

#[derive(Clone, Debug, Serialize)]
pub struct CaptureStatus {
    pub root: String,
    pub active: Option<CaptureManifest>,
    pub replay: Option<ReplayStatus>,
}

struct ActiveCapture {
    manifest: CaptureManifest,
    directory: PathBuf,
    raw_writer: BufWriter<File>,
    occupancy_writer: BufWriter<File>,
    event_writer: BufWriter<File>,
    first_packet_us: Option<u64>,
}

#[derive(Clone, Debug)]
struct RawRecord {
    offset_us: u64,
    payload: Vec<u8>,
}

pub struct CaptureManager {
    root: PathBuf,
    input_bind: String,
    forward_target: String,
    active: Option<ActiveCapture>,
    replay: Option<ReplayStatus>,
}

impl CaptureManager {
    pub fn new(
        root: impl AsRef<Path>,
        input_bind: impl Into<String>,
        forward_target: impl Into<String>,
    ) -> Result<Self> {
        let root = root.as_ref().to_path_buf();
        fs::create_dir_all(&root)
            .with_context(|| format!("failed to create capture root {}", root.display()))?;
        Ok(Self {
            root,
            input_bind: input_bind.into(),
            forward_target: forward_target.into(),
            active: None,
            replay: None,
        })
    }

    pub fn status(&self) -> CaptureStatus {
        CaptureStatus {
            root: self.root.to_string_lossy().into_owned(),
            active: self.active.as_ref().map(|active| active.manifest.clone()),
            replay: self.replay.clone(),
        }
    }

    pub fn start_capture(
        &mut self,
        label: Option<&str>,
        provenance: CaptureProvenance,
        consent: bool,
    ) -> Result<CaptureManifest> {
        if !consent {
            return Err(anyhow!("capture consent is required"));
        }
        if self.active.is_some() {
            return Err(anyhow!("a capture session is already active"));
        }
        if self.replay.as_ref().is_some_and(|replay| replay.active) {
            return Err(anyhow!("cannot capture while replay is active"));
        }
        let clean_label = label.and_then(sanitize_label);
        let mut session_id = format!("formmap-{}-{}", now_ms(), std::process::id());
        if let Some(label) = &clean_label {
            session_id.push('-');
            session_id.push_str(label);
        }
        let directory = self.root.join(&session_id);
        fs::create_dir_all(&directory)?;
        let raw_writer = BufWriter::new(File::create(directory.join(RAW_FILE_NAME))?);
        let occupancy_writer =
            BufWriter::new(File::create(directory.join(OCCUPANCY_FILE_NAME))?);
        let event_writer = BufWriter::new(File::create(directory.join(EVENTS_FILE_NAME))?);
        fs::write(
            directory.join(LAYOUT_FILE_NAME),
            serde_json::to_string_pretty(&provenance.layout)?,
        )?;
        fs::write(
            directory.join(PROVENANCE_FILE_NAME),
            serde_json::to_string_pretty(&provenance)?,
        )?;
        let manifest = CaptureManifest {
            schema: CAPTURE_SCHEMA.to_string(),
            session_id,
            label: clean_label,
            started_at_ms: now_ms(),
            ended_at_ms: None,
            source: "measured".to_string(),
            packet_count: 0,
            byte_count: 0,
            occupancy_frame_count: 0,
            event_count: 0,
            input_bind: self.input_bind.clone(),
            forward_target: self.forward_target.clone(),
            code_version: provenance.code_version,
            code_commit: provenance.code_commit,
            protocol: "ADR-018 raw UDP datagrams".to_string(),
            protocol_versions: provenance.protocol_versions,
            calibration_id: provenance.calibration_id,
            firmware: provenance.firmware,
            solver: provenance.solver,
            consent,
            files: vec![
                RAW_FILE_NAME.to_string(),
                OCCUPANCY_FILE_NAME.to_string(),
                EVENTS_FILE_NAME.to_string(),
                LAYOUT_FILE_NAME.to_string(),
                PROVENANCE_FILE_NAME.to_string(),
            ],
        };
        write_manifest(&directory, &manifest)?;
        self.active = Some(ActiveCapture {
            manifest: manifest.clone(),
            directory,
            raw_writer,
            occupancy_writer,
            event_writer,
            first_packet_us: None,
        });
        Ok(manifest)
    }

    pub fn stop_capture(&mut self) -> Result<CaptureManifest> {
        let mut active = self
            .active
            .take()
            .ok_or_else(|| anyhow!("no capture session is active"))?;
        active.raw_writer.flush()?;
        active.occupancy_writer.flush()?;
        active.event_writer.flush()?;
        active.manifest.ended_at_ms = Some(now_ms());
        write_manifest(&active.directory, &active.manifest)?;
        Ok(active.manifest)
    }

    pub fn record_datagram(&mut self, payload: &[u8]) -> Result<()> {
        let Some(active) = self.active.as_mut() else {
            return Ok(());
        };
        if payload.is_empty() || payload.len() > MAX_DATAGRAM_BYTES {
            return Err(anyhow!("invalid CSI datagram length {}", payload.len()));
        }
        let timestamp_us = now_us();
        let first = *active.first_packet_us.get_or_insert(timestamp_us);
        let offset_us = timestamp_us.saturating_sub(first);
        let length = payload.len() as u32;
        active.raw_writer.write_all(&offset_us.to_le_bytes())?;
        active.raw_writer.write_all(&length.to_le_bytes())?;
        active.raw_writer.write_all(payload)?;
        active.manifest.packet_count += 1;
        active.manifest.byte_count += payload.len() as u64;
        if active.manifest.packet_count % 200 == 0 {
            active.raw_writer.flush()?;
            write_manifest(&active.directory, &active.manifest)?;
        }
        Ok(())
    }

    pub fn record_snapshot(&mut self, system: &Value, occupancy: &Value) -> Result<()> {
        let Some(active) = self.active.as_mut() else {
            return Ok(());
        };
        serde_json::to_writer(&mut active.occupancy_writer, occupancy)?;
        active.occupancy_writer.write_all(b"\n")?;
        serde_json::to_writer(&mut active.event_writer, system)?;
        active.event_writer.write_all(b"\n")?;
        active.manifest.occupancy_frame_count += 1;
        active.manifest.event_count += 1;
        if active.manifest.occupancy_frame_count % 50 == 0 {
            active.occupancy_writer.flush()?;
            active.event_writer.flush()?;
            write_manifest(&active.directory, &active.manifest)?;
        }
        Ok(())
    }

    pub fn list_captures(&self) -> Result<Vec<CaptureManifest>> {
        let mut captures = Vec::new();
        for entry in fs::read_dir(&self.root)? {
            let entry = entry?;
            if !entry.file_type()?.is_dir() {
                continue;
            }
            let path = entry.path().join(MANIFEST_FILE_NAME);
            let Ok(text) = fs::read_to_string(path) else {
                continue;
            };
            if let Ok(manifest) = serde_json::from_str::<CaptureManifest>(&text) {
                captures.push(manifest);
            }
        }
        captures.sort_by_key(|capture| std::cmp::Reverse(capture.started_at_ms));
        Ok(captures)
    }

    pub fn get_capture(&self, capture_id: &str) -> Result<CaptureManifest> {
        let directory = self.capture_directory(capture_id)?;
        let text = fs::read_to_string(directory.join(MANIFEST_FILE_NAME))?;
        Ok(serde_json::from_str(&text)?)
    }

    pub fn delete_capture(&mut self, capture_id: &str) -> Result<()> {
        if self
            .active
            .as_ref()
            .is_some_and(|active| active.manifest.session_id == capture_id)
        {
            return Err(anyhow!("cannot delete the active capture"));
        }
        if self
            .replay
            .as_ref()
            .is_some_and(|replay| replay.active && replay.capture_id == capture_id)
        {
            return Err(anyhow!("cannot delete a capture while replaying it"));
        }
        let directory = self.capture_directory(capture_id)?;
        fs::remove_dir_all(directory)?;
        Ok(())
    }

    fn capture_directory(&self, capture_id: &str) -> Result<PathBuf> {
        validate_capture_id(capture_id)?;
        let directory = self.root.join(capture_id);
        if !directory.is_dir() {
            return Err(anyhow!("capture session not found: {capture_id}"));
        }
        let canonical_root = self.root.canonicalize()?;
        let canonical_directory = directory.canonicalize()?;
        if !canonical_directory.starts_with(&canonical_root) {
            return Err(anyhow!("capture path escapes capture root"));
        }
        Ok(canonical_directory)
    }
}

pub fn start_capture_proxy(
    input_bind: &str,
    forward_target: &str,
    root: &str,
) -> Result<SharedCapture> {
    let manager = Arc::new(Mutex::new(CaptureManager::new(
        root,
        input_bind,
        forward_target,
    )?));
    let socket = UdpSocket::bind(input_bind)
        .with_context(|| format!("failed to bind public CSI capture socket {input_bind}"))?;
    socket.set_read_timeout(Some(Duration::from_secs(1)))?;
    let forward_socket = UdpSocket::bind("0.0.0.0:0")?;
    let target = forward_target.to_string();
    let capture = manager.clone();
    std::thread::Builder::new()
        .name("formmap-capture-proxy".to_string())
        .spawn(move || {
            let mut buffer = [0u8; MAX_DATAGRAM_BYTES];
            loop {
                match socket.recv_from(&mut buffer) {
                    Ok((length, _source)) => {
                        if let Ok(mut manager) = capture.lock() {
                            if let Err(error) = manager.record_datagram(&buffer[..length]) {
                                eprintln!("FormMap capture write error: {error}");
                            }
                        }
                        if let Err(error) = forward_socket.send_to(&buffer[..length], &target) {
                            eprintln!("FormMap CSI forward error: {error}");
                        }
                    }
                    Err(error)
                        if error.kind() == ErrorKind::WouldBlock
                            || error.kind() == ErrorKind::TimedOut => {}
                    Err(error) => eprintln!("FormMap capture socket error: {error}"),
                }
            }
        })?;
    Ok(manager)
}

pub fn spawn_replay(
    shared: SharedCapture,
    capture_id: &str,
    target: &str,
    speed: f32,
) -> Result<ReplayStatus> {
    if !speed.is_finite() || !(0.05..=100.0).contains(&speed) {
        return Err(anyhow!("replay speed must be within 0.05..100"));
    }
    let directory = {
        let manager = shared
            .lock()
            .map_err(|_| anyhow!("capture state lock poisoned"))?;
        if manager.active.is_some() {
            return Err(anyhow!("stop the active capture before replay"));
        }
        if manager.replay.as_ref().is_some_and(|status| status.active) {
            return Err(anyhow!("a replay is already active"));
        }
        manager.capture_directory(capture_id)?
    };
    let records = read_records(&directory.join(RAW_FILE_NAME))?;
    if records.is_empty() {
        return Err(anyhow!("capture contains no CSI datagrams"));
    }
    let status = ReplayStatus {
        capture_id: capture_id.to_string(),
        active: true,
        speed,
        packet_count: records.len(),
        packets_sent: 0,
        started_at_ms: now_ms(),
        completed_at_ms: None,
        error: None,
    };
    {
        let mut manager = shared
            .lock()
            .map_err(|_| anyhow!("capture state lock poisoned"))?;
        manager.replay = Some(status.clone());
    }

    let capture_id_owned = capture_id.to_string();
    let target_owned = target.to_string();
    std::thread::Builder::new()
        .name("formmap-replay".to_string())
        .spawn(move || {
            let result = replay_records(&records, &target_owned, speed, |sent| {
                if let Ok(mut manager) = shared.lock() {
                    if let Some(replay) = manager.replay.as_mut() {
                        replay.packets_sent = sent;
                    }
                }
            });
            if let Ok(mut manager) = shared.lock() {
                let replay = manager.replay.get_or_insert(ReplayStatus {
                    capture_id: capture_id_owned,
                    active: false,
                    speed,
                    packet_count: records.len(),
                    packets_sent: 0,
                    started_at_ms: now_ms(),
                    completed_at_ms: None,
                    error: None,
                });
                replay.active = false;
                replay.completed_at_ms = Some(now_ms());
                if let Err(error) = result {
                    replay.error = Some(error.to_string());
                }
            }
        })?;
    Ok(status)
}

fn replay_records<F>(records: &[RawRecord], target: &str, speed: f32, mut progress: F) -> Result<()>
where
    F: FnMut(usize),
{
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    let started = Instant::now();
    for (index, record) in records.iter().enumerate() {
        let target_offset = Duration::from_micros((record.offset_us as f64 / speed as f64) as u64);
        if let Some(wait) = target_offset.checked_sub(started.elapsed()) {
            std::thread::sleep(wait);
        }
        socket.send_to(&record.payload, target)?;
        progress(index + 1);
    }
    Ok(())
}

fn read_records(path: &Path) -> Result<Vec<RawRecord>> {
    let mut reader = BufReader::new(
        File::open(path).with_context(|| format!("failed to open {}", path.display()))?,
    );
    let mut records = Vec::new();
    loop {
        let mut header = [0u8; 12];
        match reader.read_exact(&mut header) {
            Ok(()) => {}
            Err(error) if error.kind() == ErrorKind::UnexpectedEof => break,
            Err(error) => return Err(error.into()),
        }
        let offset_us = u64::from_le_bytes(header[0..8].try_into().unwrap());
        let length = u32::from_le_bytes(header[8..12].try_into().unwrap()) as usize;
        if length == 0 || length > MAX_DATAGRAM_BYTES {
            return Err(anyhow!("invalid captured datagram length {length}"));
        }
        let mut payload = vec![0u8; length];
        reader.read_exact(&mut payload)?;
        records.push(RawRecord { offset_us, payload });
    }
    Ok(records)
}

fn write_manifest(directory: &Path, manifest: &CaptureManifest) -> Result<()> {
    let text = serde_json::to_string_pretty(manifest)?;
    fs::write(directory.join(MANIFEST_FILE_NAME), text)?;
    Ok(())
}

fn sanitize_label(value: &str) -> Option<String> {
    let cleaned = value
        .chars()
        .filter(|character| character.is_ascii_alphanumeric() || matches!(character, '-' | '_'))
        .take(40)
        .collect::<String>();
    (!cleaned.is_empty()).then_some(cleaned)
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
    fn raw_capture_roundtrip_preserves_datagrams_and_provenance() {
        let root = std::env::temp_dir().join(format!("formmap-capture-test-{}", now_us()));
        let mut manager = CaptureManager::new(&root, "0.0.0.0:3333", "127.0.0.1:3334").unwrap();
        let manifest = manager
            .start_capture(
                Some("room-a"),
                CaptureProvenance {
                    calibration_id: "cal-test".to_string(),
                    ..CaptureProvenance::default()
                },
                true,
            )
            .unwrap();
        manager.record_datagram(&[1, 2, 3, 4]).unwrap();
        manager.record_datagram(&[9, 8, 7]).unwrap();
        manager
            .record_snapshot(
                &serde_json::json!({"timestamp_ms": 1, "state": "READY"}),
                &serde_json::json!({"timestamp": 1, "cells": []}),
            )
            .unwrap();
        let stopped = manager.stop_capture().unwrap();
        assert_eq!(stopped.packet_count, 2);
        assert_eq!(stopped.occupancy_frame_count, 1);
        assert_eq!(stopped.calibration_id, "cal-test");
        let records = read_records(&root.join(&manifest.session_id).join(RAW_FILE_NAME)).unwrap();
        assert_eq!(records.len(), 2);
        assert_eq!(records[0].payload, vec![1, 2, 3, 4]);
        assert_eq!(records[1].payload, vec![9, 8, 7]);
        let _ = fs::remove_dir_all(root);
    }

    #[test]
    fn capture_id_rejects_path_traversal() {
        assert!(validate_capture_id("../outside").is_err());
        assert!(validate_capture_id("valid-session_1").is_ok());
    }

    #[test]
    fn capture_requires_consent() {
        let root = std::env::temp_dir().join(format!("formmap-consent-test-{}", now_us()));
        let mut manager = CaptureManager::new(&root, "0.0.0.0:3333", "127.0.0.1:3334").unwrap();
        assert!(manager
            .start_capture(None, CaptureProvenance::default(), false)
            .is_err());
        let _ = fs::remove_dir_all(root);
    }
}

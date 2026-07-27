//! WiFi FormMap runtime.
//!
//! Responsibilities in this crate are deliberately limited to the point-cloud adapter runtime:
//! station-aware ingest, bounded packet synchronization, robust empty-room calibration,
//! link-quality gating, RTI occupancy output and the Link Agent compatibility protocol.

use crate::baseline::{
    BaselineConfig, BaselineEngine, BaselineInvalidationReason, BaselineSummary, LinkObservation,
};
use crate::parser::{parse_adr018, stable_station_id_from_mac_hash, CsiFrame};
use crate::rti::{RtiConfig, RtiEngine, RtiLinkMeasurement, SolverComparison};
use crate::synchronizer::{LinkSynchronizer, SequenceDiagnostics};
use anyhow::{anyhow, Result};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet, VecDeque};
use std::fs;
use std::net::{SocketAddr, UdpSocket};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

pub type SharedState = Arc<Mutex<FormMapState>>;

const DEFAULT_CSI_TIMEOUT_MS: u64 = 3_000;
const DEFAULT_AGENT_TIMEOUT_MS: u64 = 5_000;
const PROBE_CORRELATION_WINDOW_MS: u64 = 180;
const MAX_HISTORY: usize = 96;
const MIN_READY_LINKS: usize = 3;

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

fn default_true() -> bool {
    true
}

fn default_room() -> [f32; 3] {
    [4.0, 2.4, 4.0]
}

fn default_grid() -> [usize; 3] {
    [20, 10, 20]
}

fn default_threshold() -> f32 {
    0.24
}

fn default_fresnel_width() -> f32 {
    0.65
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NodePlacement {
    pub node_id: u8,
    pub name: String,
    pub position_m: [f32; 3],
    #[serde(default = "default_true")]
    pub enabled: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StationPlacement {
    pub station_id: u16,
    pub name: String,
    pub position_m: [f32; 3],
    #[serde(default = "default_true")]
    pub enabled: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RoomLayout {
    #[serde(default = "default_room")]
    pub room_size_m: [f32; 3],
    #[serde(default = "default_grid")]
    pub grid_size: [usize; 3],
    #[serde(default = "default_threshold")]
    pub threshold: f32,
    #[serde(default = "default_fresnel_width")]
    pub fresnel_width_m: f32,
    #[serde(default)]
    pub baseline: BaselineConfig,
    #[serde(default)]
    pub rti: RtiConfig,
    #[serde(default)]
    pub nodes: Vec<NodePlacement>,
    #[serde(default)]
    pub stations: Vec<StationPlacement>,
}

impl Default for RoomLayout {
    fn default() -> Self {
        let mut rti = RtiConfig::default();
        rti.fresnel_width_m = default_fresnel_width();
        Self {
            room_size_m: default_room(),
            grid_size: default_grid(),
            threshold: default_threshold(),
            fresnel_width_m: default_fresnel_width(),
            baseline: BaselineConfig::default(),
            rti,
            nodes: Vec::new(),
            stations: Vec::new(),
        }
    }
}

impl RoomLayout {
    pub fn validate(&self) -> Result<()> {
        if self
            .room_size_m
            .iter()
            .any(|value| !value.is_finite() || *value <= 0.2 || *value > 100.0)
        {
            return Err(anyhow!(
                "room dimensions must be finite and within 0.2..100 m"
            ));
        }
        if self.grid_size.iter().any(|value| *value < 2 || *value > 64) {
            return Err(anyhow!("grid dimensions must be within 2..64"));
        }
        let cells = self.grid_size[0] * self.grid_size[1] * self.grid_size[2];
        if cells > 100_000 {
            return Err(anyhow!("grid is too large"));
        }
        if !(0.01..=0.95).contains(&self.threshold) {
            return Err(anyhow!("threshold must be within 0.01..0.95"));
        }
        if !(0.05..=5.0).contains(&self.fresnel_width_m) {
            return Err(anyhow!("fresnel_width_m must be within 0.05..5.0"));
        }
        for node in &self.nodes {
            validate_position(node.position_m, self.room_size_m)?;
        }
        for station in &self.stations {
            validate_position(station.position_m, self.room_size_m)?;
        }
        Ok(())
    }

    pub fn node_position(&self, node_id: u8) -> Option<[f32; 3]> {
        self.nodes
            .iter()
            .find(|node| node.node_id == node_id && node.enabled)
            .map(|node| node.position_m)
    }

    pub fn station_position(&self, station_id: u16) -> Option<[f32; 3]> {
        self.stations
            .iter()
            .find(|station| station.station_id == station_id && station.enabled)
            .map(|station| station.position_m)
    }

    fn ensure_node(&mut self, node_id: u8) {
        if self.nodes.iter().any(|node| node.node_id == node_id) {
            return;
        }
        let index = self.nodes.len();
        let [width, height, depth] = self.room_size_m;
        let candidates = [
            [0.15, height * 0.55, 0.15],
            [width - 0.15, height * 0.55, depth - 0.15],
            [width - 0.15, height * 0.78, 0.15],
            [0.15, height * 0.30, depth - 0.15],
        ];
        self.nodes.push(NodePlacement {
            node_id,
            name: format!("ESP32 Node {node_id}"),
            position_m: candidates[index % candidates.len()],
            enabled: true,
        });
    }

    fn ensure_station(&mut self, station_id: u16, name: Option<&str>) {
        if let Some(station) = self
            .stations
            .iter_mut()
            .find(|station| station.station_id == station_id)
        {
            if let Some(name) = name {
                station.name = name.to_string();
            }
            return;
        }
        let index = self.stations.len();
        let [width, height, depth] = self.room_size_m;
        let candidates = [
            [width - 0.25, height * 0.35, 0.25],
            [0.25, height * 0.72, depth - 0.25],
            [width - 0.25, height * 0.55, depth - 0.25],
            [0.25, height * 0.25, 0.25],
            [width * 0.5, height * 0.82, depth - 0.2],
        ];
        self.stations.push(StationPlacement {
            station_id,
            name: name
                .map(ToString::to_string)
                .unwrap_or_else(|| format!("Station {station_id}")),
            position_m: candidates[index % candidates.len()],
            enabled: true,
        });
    }

    fn geometry_changes(&self, replacement: &RoomLayout) -> (HashSet<u8>, HashSet<u16>, bool) {
        let mut nodes = HashSet::new();
        let mut stations = HashSet::new();
        for old in &self.nodes {
            match replacement.nodes.iter().find(|new| new.node_id == old.node_id) {
                Some(new)
                    if same_position(old.position_m, new.position_m)
                        && old.enabled == new.enabled => {}
                _ => {
                    nodes.insert(old.node_id);
                }
            }
        }
        for new in &replacement.nodes {
            if !self.nodes.iter().any(|old| old.node_id == new.node_id) {
                nodes.insert(new.node_id);
            }
        }
        for old in &self.stations {
            match replacement
                .stations
                .iter()
                .find(|new| new.station_id == old.station_id)
            {
                Some(new)
                    if same_position(old.position_m, new.position_m)
                        && old.enabled == new.enabled => {}
                _ => {
                    stations.insert(old.station_id);
                }
            }
        }
        for new in &replacement.stations {
            if !self
                .stations
                .iter()
                .any(|old| old.station_id == new.station_id)
            {
                stations.insert(new.station_id);
            }
        }
        let room_changed = !same_position(self.room_size_m, replacement.room_size_m)
            || self.grid_size != replacement.grid_size
            || (self.fresnel_width_m - replacement.fresnel_width_m).abs() > 1.0e-4;
        (nodes, stations, room_changed)
    }
}

fn validate_position(position: [f32; 3], room: [f32; 3]) -> Result<()> {
    if position.iter().any(|value| !value.is_finite()) {
        return Err(anyhow!("device positions must be finite"));
    }
    for axis in 0..3 {
        if position[axis] < 0.0 || position[axis] > room[axis] {
            return Err(anyhow!("device position is outside the room"));
        }
    }
    Ok(())
}

fn same_position(left: [f32; 3], right: [f32; 3]) -> bool {
    left.into_iter()
        .zip(right)
        .all(|(left, right)| (left - right).abs() <= 0.01)
}

#[derive(Clone, Copy, Debug, Hash, Eq, PartialEq, Serialize)]
pub struct LinkKey {
    pub node_id: u8,
    pub station_id: u16,
}

impl LinkKey {
    pub fn numeric_id(self) -> u32 {
        ((self.node_id as u32) << 16) | self.station_id as u32
    }
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct LinkQualityFactors {
    pub packet_quality: f32,
    pub stationarity_score: f32,
    pub rssi_quality: f32,
    pub phase_stability: f32,
    pub channel_consistency: f32,
    pub baseline_quality: f32,
    pub final_quality: f32,
}

#[derive(Clone, Debug, Serialize)]
pub struct LinkSnapshot {
    pub link_id: u32,
    pub node_id: u8,
    pub station_id: u16,
    pub protocol_version: u8,
    pub firmware_version_code: u16,
    pub frame_count: u64,
    pub last_seen_ms: u64,
    pub online: bool,
    pub channel: u8,
    pub rssi_dbm: f32,
    pub noise_floor_dbm: f32,
    pub sample_rate_hz: f32,
    pub packet_loss: f32,
    pub amplitude_mean: f32,
    pub motion_score: f32,
    pub observation: LinkObservation,
    pub quality: LinkQualityFactors,
    pub baseline: BaselineSummary,
    pub synchronization: SequenceDiagnostics,
    pub source: String,
    pub source_mac_hash: String,
}

struct LinkState {
    key: LinkKey,
    protocol_version: u8,
    firmware_version_code: u16,
    frame_count: u64,
    first_seen_ms: u64,
    last_seen_ms: u64,
    channel: u8,
    rssi_dbm: f32,
    noise_floor_dbm: f32,
    amplitude_mean: f32,
    history: VecDeque<f32>,
    motion_score: f32,
    stationarity_score: f32,
    observation: LinkObservation,
    quality: LinkQualityFactors,
    source: String,
    source_mac_hash: [u8; 8],
    synchronizer: LinkSynchronizer,
    baseline: BaselineEngine,
}

impl LinkState {
    fn new(
        key: LinkKey,
        source: &str,
        now: u64,
        baseline_config: BaselineConfig,
        stationarity_score: f32,
    ) -> Self {
        Self {
            key,
            protocol_version: 1,
            firmware_version_code: 0,
            frame_count: 0,
            first_seen_ms: now,
            last_seen_ms: now,
            channel: 0,
            rssi_dbm: -100.0,
            noise_floor_dbm: -100.0,
            amplitude_mean: 0.0,
            history: VecDeque::with_capacity(MAX_HISTORY),
            motion_score: 0.0,
            stationarity_score,
            observation: LinkObservation::default(),
            quality: LinkQualityFactors::default(),
            source: source.to_string(),
            source_mac_hash: [0; 8],
            synchronizer: LinkSynchronizer::default(),
            baseline: BaselineEngine::new(baseline_config),
        }
    }

    fn ingest(&mut self, mut frame: CsiFrame, stationarity_score: f32) {
        self.stationarity_score = stationarity_score.clamp(0.0, 1.0);
        // ADR-018 v2 firmware maintains a per-station sequence in agent_sequence.
        // This prevents other stations on the same receiver from appearing as gaps.
        if frame.protocol_version == 2 && frame.agent_sequence != 0 {
            frame.sequence = frame.agent_sequence;
        }
        let synchronized = self.synchronizer.push(frame);
        for synchronized in synchronized {
            self.process_ordered_frame(synchronized.frame);
        }
    }

    fn process_ordered_frame(&mut self, frame: CsiFrame) {
        if self.firmware_version_code != 0
            && frame.firmware_version_code != 0
            && self.firmware_version_code != frame.firmware_version_code
        {
            self.baseline
                .invalidate(BaselineInvalidationReason::FirmwareChanged);
        }
        self.protocol_version = frame.protocol_version;
        self.firmware_version_code = frame.firmware_version_code;
        self.frame_count += 1;
        self.last_seen_ms = frame.received_at_ms;
        self.channel = frame.channel;
        self.rssi_dbm = frame.rssi as f32;
        self.noise_floor_dbm = frame.noise_floor as f32;
        self.source_mac_hash = frame.source_mac_hash;
        self.amplitude_mean = mean(&frame.amplitudes);
        self.history.push_back(self.amplitude_mean);
        while self.history.len() > MAX_HISTORY {
            self.history.pop_front();
        }
        self.motion_score = normalized_variance(&self.history);

        let synchronization = self.synchronizer.diagnostics();
        let expected = synchronization
            .delivered
            .saturating_add(synchronization.missing_packets)
            .saturating_add(synchronization.duplicates)
            .saturating_add(synchronization.late_packets);
        let packet_quality = if expected == 0 {
            0.0
        } else {
            synchronization.delivered as f32 / expected as f32
        }
        .clamp(0.0, 1.0);

        self.observation = self.baseline.ingest(
            &frame,
            packet_quality,
            self.stationarity_score,
            self.motion_score,
        );
        let snr = (self.rssi_dbm - self.noise_floor_dbm).max(0.0);
        let rssi_quality = (snr / 35.0).clamp(0.0, 1.0);
        let baseline_summary = self.baseline.summary();
        let channel_consistency = match baseline_summary.channel {
            Some(channel) if channel == frame.channel => 1.0,
            Some(_) => 0.0,
            None => 0.5,
        };
        let phase_stability = baseline_summary.phase_stability.clamp(0.0, 1.0);
        let baseline_quality = baseline_summary.quality.clamp(0.0, 1.0);
        let product = packet_quality
            * self.stationarity_score
            * rssi_quality
            * phase_stability.max(0.05)
            * channel_consistency
            * baseline_quality.max(0.05);
        self.quality = LinkQualityFactors {
            packet_quality,
            stationarity_score: self.stationarity_score,
            rssi_quality,
            phase_stability,
            channel_consistency,
            baseline_quality,
            final_quality: product.powf(1.0 / 6.0).clamp(0.0, 1.0),
        };
    }

    fn invalidate(&mut self, reason: BaselineInvalidationReason) {
        self.baseline.invalidate(reason);
        self.observation = LinkObservation::default();
        self.quality.baseline_quality = 0.0;
        self.quality.final_quality = 0.0;
    }

    fn ready_for_solver(&self, now: u64, minimum_quality: f32) -> bool {
        now.saturating_sub(self.last_seen_ms) <= DEFAULT_CSI_TIMEOUT_MS
            && self.baseline.ready()
            && self.quality.final_quality >= minimum_quality
            && self.stationarity_score >= 0.75
    }

    fn snapshot(&self, now: u64) -> LinkSnapshot {
        let elapsed_s =
            ((self.last_seen_ms.saturating_sub(self.first_seen_ms)) as f32 / 1000.0).max(0.1);
        let sample_rate_hz = self.frame_count as f32 / elapsed_s;
        let synchronization = self.synchronizer.diagnostics();
        let expected = synchronization
            .delivered
            .saturating_add(synchronization.missing_packets);
        let packet_loss = if expected == 0 {
            0.0
        } else {
            synchronization.missing_packets as f32 / expected as f32
        };
        LinkSnapshot {
            link_id: self.key.numeric_id(),
            node_id: self.key.node_id,
            station_id: self.key.station_id,
            protocol_version: self.protocol_version,
            firmware_version_code: self.firmware_version_code,
            frame_count: self.frame_count,
            last_seen_ms: self.last_seen_ms,
            online: now.saturating_sub(self.last_seen_ms) <= DEFAULT_CSI_TIMEOUT_MS,
            channel: self.channel,
            rssi_dbm: self.rssi_dbm,
            noise_floor_dbm: self.noise_floor_dbm,
            sample_rate_hz,
            packet_loss,
            amplitude_mean: self.amplitude_mean,
            motion_score: self.motion_score,
            observation: self.observation.clone(),
            quality: self.quality.clone(),
            baseline: self.baseline.summary(),
            synchronization,
            source: self.source.clone(),
            source_mac_hash: hex_hash(&self.source_mac_hash),
        }
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct NodeRuntime {
    pub node_id: u8,
    pub last_seen_ms: u64,
    pub online: bool,
    pub channel: u8,
    pub protocol_version: u8,
    pub firmware_version_code: u16,
}

#[derive(Clone, Debug, Serialize)]
pub struct StationRuntime {
    pub station_id: u16,
    pub name: String,
    pub platform: String,
    pub address: String,
    pub last_seen_ms: u64,
    pub last_sequence: u32,
    pub online: bool,
    pub stationarity_score: f32,
    pub moved_at_ms: Option<u64>,
    pub nic: Option<String>,
    pub ssid: Option<String>,
    pub bssid: Option<String>,
    pub rssi_dbm: Option<f32>,
    pub link_speed_mbps: Option<f32>,
    pub thermal_c: Option<f32>,
    pub source_mac_hash: Option<[u8; 8]>,
}

#[derive(Clone, Debug)]
struct ProbeEvent {
    station_id: u16,
    sequence: u32,
    received_at_ms: u64,
}

#[derive(Clone, Debug, Serialize)]
pub struct SparseCell {
    pub x: usize,
    pub y: usize,
    pub z: usize,
    pub probability: f32,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct HeightLayerSummary {
    pub low: f32,
    pub mid: f32,
    pub high: f32,
    pub dominant: String,
}

#[derive(Clone, Debug, Serialize)]
pub struct OccupancyFrame {
    pub schema: String,
    pub frame_id: u64,
    pub timestamp_ms: u64,
    pub grid_size: [usize; 3],
    pub room_size_m: [f32; 3],
    pub cells: Vec<SparseCell>,
    pub centroid_m: Option<[f32; 3]>,
    pub extent_m: Option<[f32; 3]>,
    pub confidence: f32,
    pub active_links: usize,
    pub calibrated_links: usize,
    pub calibration_id: String,
    pub height_layers: HeightLayerSummary,
    pub solver: SolverComparison,
    pub degraded: bool,
    pub note: String,
}

impl OccupancyFrame {
    fn empty(layout: &RoomLayout) -> Self {
        Self {
            schema: "formmap-occupancy/v1".to_string(),
            frame_id: 0,
            timestamp_ms: now_ms(),
            grid_size: layout.grid_size,
            room_size_m: layout.room_size_m,
            cells: Vec::new(),
            centroid_m: None,
            extent_m: None,
            confidence: 0.0,
            active_links: 0,
            calibrated_links: 0,
            calibration_id: "unavailable".to_string(),
            height_layers: HeightLayerSummary::default(),
            solver: SolverComparison::default(),
            degraded: true,
            note: "RF probability volume; not a camera image or calibrated body mesh"
                .to_string(),
        }
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct BlockedLink {
    pub link_id: u32,
    pub reasons: Vec<String>,
}

#[derive(Clone, Debug, Serialize)]
pub struct CalibrationStatus {
    pub collecting: bool,
    pub target_samples_per_link: usize,
    pub total_links: usize,
    pub ready_links: usize,
    pub progress: f32,
    pub minimum_quality: f32,
    pub minimum_ready_links: usize,
    pub blocked_links: Vec<BlockedLink>,
    pub drift_warning_links: usize,
    pub recalibration_required: bool,
    pub calibration_id: String,
}

#[derive(Clone, Debug, Serialize)]
pub struct FormMapSnapshot {
    pub mode: String,
    pub system_state: String,
    pub reason_code: String,
    pub recovery: String,
    pub ready: bool,
    pub sensor_online: bool,
    pub csi_frames_received: u64,
    pub last_csi_ms: Option<u64>,
    pub connected_nodes: usize,
    pub connected_stations: usize,
    pub active_links: usize,
    pub drift_index: f32,
    pub recalibration_required: bool,
    pub simulation_enabled: bool,
    pub camera_enabled: bool,
    pub skeleton_enabled: bool,
    pub calibration: CalibrationStatus,
    pub nodes: Vec<NodeRuntime>,
    pub links: Vec<LinkSnapshot>,
    pub occupancy: OccupancyFrame,
    pub layout: RoomLayout,
}

pub struct FormMapState {
    pub layout: RoomLayout,
    layout_path: PathBuf,
    links: HashMap<LinkKey, LinkState>,
    stations: HashMap<u16, StationRuntime>,
    station_by_name: HashMap<String, u16>,
    station_by_hash: HashMap<[u8; 8], u16>,
    recent_probes: VecDeque<ProbeEvent>,
    next_station_id: u16,
    total_frames: u64,
    last_csi_ms: Option<u64>,
    occupancy: OccupancyFrame,
    occupancy_frame_id: u64,
    rti: RtiEngine,
}

impl FormMapState {
    pub fn load(layout_path: impl AsRef<Path>) -> Self {
        let path = layout_path.as_ref().to_path_buf();
        let layout = fs::read_to_string(&path)
            .ok()
            .and_then(|text| serde_json::from_str::<RoomLayout>(&text).ok())
            .filter(|layout| layout.validate().is_ok())
            .unwrap_or_default();
        Self::from_layout(path, layout)
    }

    fn from_layout(path: PathBuf, mut layout: RoomLayout) -> Self {
        layout.baseline = layout.baseline.clone().normalized();
        layout.rti = layout.rti.clone().normalized();
        layout.rti.fresnel_width_m = layout.fresnel_width_m;
        let next_station_id = layout
            .stations
            .iter()
            .map(|station| station.station_id)
            .max()
            .unwrap_or(0)
            .saturating_add(1)
            .max(1);
        Self {
            occupancy: OccupancyFrame::empty(&layout),
            rti: RtiEngine::new(layout.rti.clone()),
            layout,
            layout_path: path,
            links: HashMap::new(),
            stations: HashMap::new(),
            station_by_name: HashMap::new(),
            station_by_hash: HashMap::new(),
            recent_probes: VecDeque::with_capacity(512),
            next_station_id,
            total_frames: 0,
            last_csi_ms: None,
            occupancy_frame_id: 0,
        }
    }

    pub fn save_layout(&self) -> Result<()> {
        let text = serde_json::to_string_pretty(&self.layout)?;
        fs::write(&self.layout_path, text)?;
        Ok(())
    }

    pub fn replace_layout(&mut self, mut replacement: RoomLayout) -> Result<()> {
        replacement.validate()?;
        replacement.baseline = replacement.baseline.clone().normalized();
        replacement.rti = replacement.rti.clone().normalized();
        replacement.rti.fresnel_width_m = replacement.fresnel_width_m;
        let (changed_nodes, changed_stations, room_changed) =
            self.layout.geometry_changes(&replacement);
        for link in self.links.values_mut() {
            if room_changed {
                link.invalidate(BaselineInvalidationReason::LayoutChanged);
            } else if changed_nodes.contains(&link.key.node_id) {
                link.invalidate(BaselineInvalidationReason::NodeMoved);
            } else if changed_stations.contains(&link.key.station_id) {
                link.invalidate(BaselineInvalidationReason::StationMoved);
            }
        }
        self.layout = replacement;
        self.rti = RtiEngine::new(self.layout.rti.clone());
        self.occupancy = OccupancyFrame::empty(&self.layout);
        self.save_layout()
    }

    pub fn reset_calibration(&mut self) {
        for link in self.links.values_mut() {
            link.invalidate(BaselineInvalidationReason::ManualReset);
        }
        self.rti.invalidate_geometry();
        self.occupancy = OccupancyFrame::empty(&self.layout);
    }

    pub fn invalidate_node(&mut self, node_id: u8, reason: BaselineInvalidationReason) {
        for link in self.links.values_mut() {
            if link.key.node_id == node_id {
                link.invalidate(reason.clone());
            }
        }
        self.rti.invalidate_geometry();
    }

    pub fn invalidate_station(&mut self, station_id: u16, reason: BaselineInvalidationReason) {
        for link in self.links.values_mut() {
            if link.key.station_id == station_id {
                link.invalidate(reason.clone());
            }
        }
        self.rti.invalidate_geometry();
    }

    fn assign_station(
        &mut self,
        name: &str,
        platform: &str,
        source: SocketAddr,
        source_mac_hash: Option<[u8; 8]>,
        nic: Option<String>,
        ssid: Option<String>,
        bssid: Option<String>,
    ) -> u16 {
        let existing = source_mac_hash
            .and_then(|hash| self.station_by_hash.get(&hash).copied())
            .or_else(|| self.station_by_name.get(name).copied());
        if let Some(id) = existing {
            if let Some(station) = self.stations.get_mut(&id) {
                station.platform = platform.to_string();
                station.address = source.to_string();
                station.last_seen_ms = now_ms();
                station.online = true;
                station.nic = nic;
                station.ssid = ssid;
                station.bssid = bssid;
                if source_mac_hash.is_some() {
                    station.source_mac_hash = source_mac_hash;
                }
            }
            self.layout.ensure_station(id, Some(name));
            return id;
        }

        let preferred = source_mac_hash
            .map(|hash| stable_station_id_from_mac_hash(&hash))
            .filter(|id| !self.stations.contains_key(id));
        let id = preferred.unwrap_or_else(|| {
            let id = self.next_station_id.max(1);
            self.next_station_id = self.next_station_id.saturating_add(1).max(1);
            id
        });
        self.station_by_name.insert(name.to_string(), id);
        if let Some(hash) = source_mac_hash {
            self.station_by_hash.insert(hash, id);
        }
        self.stations.insert(
            id,
            StationRuntime {
                station_id: id,
                name: name.to_string(),
                platform: platform.to_string(),
                address: source.to_string(),
                last_seen_ms: now_ms(),
                last_sequence: 0,
                online: true,
                stationarity_score: 1.0,
                moved_at_ms: None,
                nic,
                ssid,
                bssid,
                rssi_dbm: None,
                link_speed_mbps: None,
                thermal_c: None,
                source_mac_hash,
            },
        );
        self.layout.ensure_station(id, Some(name));
        let _ = self.save_layout();
        id
    }

    fn record_probe(
        &mut self,
        station_id: u16,
        sequence: u32,
        source: SocketAddr,
        stationary: bool,
        stationarity_score: Option<f32>,
        rssi_dbm: Option<f32>,
        link_speed_mbps: Option<f32>,
        thermal_c: Option<f32>,
    ) {
        let now = now_ms();
        let score = stationarity_score
            .unwrap_or(if stationary { 1.0 } else { 0.0 })
            .clamp(0.0, 1.0);
        let mut moved = false;
        if let Some(station) = self.stations.get_mut(&station_id) {
            moved = station.stationarity_score >= 0.75 && score < 0.50;
            station.last_seen_ms = now;
            station.last_sequence = sequence;
            station.address = source.to_string();
            station.online = true;
            station.stationarity_score = station.stationarity_score * 0.8 + score * 0.2;
            station.rssi_dbm = rssi_dbm;
            station.link_speed_mbps = link_speed_mbps;
            station.thermal_c = thermal_c;
            if moved {
                station.moved_at_ms = Some(now);
            }
        }
        if moved {
            self.invalidate_station(station_id, BaselineInvalidationReason::StationMoved);
        }
        if score >= 0.75 {
            self.recent_probes.push_back(ProbeEvent {
                station_id,
                sequence,
                received_at_ms: now,
            });
        }
        while self.recent_probes.len() > 512 {
            self.recent_probes.pop_front();
        }
        while self
            .recent_probes
            .front()
            .is_some_and(|event| now.saturating_sub(event.received_at_ms) > 2_000)
        {
            self.recent_probes.pop_front();
        }
    }

    fn correlate_station(&self, csi_received_ms: u64) -> Option<(u16, u32)> {
        self.recent_probes
            .iter()
            .filter_map(|event| {
                let delta = csi_received_ms.abs_diff(event.received_at_ms);
                (delta <= PROBE_CORRELATION_WINDOW_MS)
                    .then_some((delta, event.station_id, event.sequence))
            })
            .min_by_key(|item| item.0)
            .map(|(_, station_id, sequence)| (station_id, sequence))
    }

    pub fn ingest_frame(&mut self, mut frame: CsiFrame) {
        self.total_frames += 1;
        self.last_csi_ms = Some(frame.received_at_ms);
        self.layout.ensure_node(frame.node_id);

        let source = if frame.station_id != 0 {
            if frame.source_mac_hash != [0; 8] {
                self.station_by_hash
                    .entry(frame.source_mac_hash)
                    .or_insert(frame.station_id);
            }
            "adr018-v2"
        } else if let Some((station_id, agent_sequence)) =
            self.correlate_station(frame.received_at_ms)
        {
            frame.station_id = station_id;
            frame.agent_sequence = agent_sequence;
            frame.link_id = ((frame.node_id as u32) << 16) | station_id as u32;
            "agent-time-correlation"
        } else {
            frame.station_id = frame.node_id as u16;
            frame.link_id = ((frame.node_id as u32) << 16) | frame.station_id as u32;
            "legacy-node-link"
        };

        self.layout.ensure_station(frame.station_id, None);
        let stationarity_score = self
            .stations
            .get(&frame.station_id)
            .map_or(1.0, |station| station.stationarity_score);
        let key = LinkKey {
            node_id: frame.node_id,
            station_id: frame.station_id,
        };
        let baseline_config = self.layout.baseline.clone();
        self.links
            .entry(key)
            .or_insert_with(|| {
                LinkState::new(
                    key,
                    source,
                    frame.received_at_ms,
                    baseline_config,
                    stationarity_score,
                )
            })
            .ingest(frame, stationarity_score);
    }

    pub fn solve_occupancy(&mut self) {
        let now = now_ms();
        for station in self.stations.values_mut() {
            station.online = now.saturating_sub(station.last_seen_ms) <= DEFAULT_AGENT_TIMEOUT_MS;
        }
        let measurements = self
            .links
            .values()
            .filter(|link| {
                link.ready_for_solver(now, self.layout.rti.minimum_link_quality)
            })
            .filter_map(|link| {
                let transmitter_m = self.layout.station_position(link.key.station_id)?;
                let receiver_m = self.layout.node_position(link.key.node_id)?;
                Some(RtiLinkMeasurement {
                    link_id: link.key.numeric_id(),
                    transmitter_m,
                    receiver_m,
                    observation: link.observation.combined,
                    quality: link.quality.final_quality,
                })
            })
            .collect::<Vec<_>>();
        let volume = self
            .rti
            .solve(self.layout.room_size_m, self.layout.grid_size, &measurements);

        self.occupancy_frame_id += 1;
        let [nx, ny, nz] = volume.grid_size;
        let mut cells = Vec::new();
        let mut weighted = [0.0f32; 3];
        let mut weight_sum = 0.0f32;
        let mut minimum = [f32::MAX; 3];
        let mut maximum = [f32::MIN; 3];
        for z in 0..nz {
            for y in 0..ny {
                for x in 0..nx {
                    let index = z * ny * nx + y * nx + x;
                    let probability = volume.dense.get(index).copied().unwrap_or(0.0);
                    if probability < self.layout.threshold {
                        continue;
                    }
                    let center = [
                        (x as f32 + 0.5) * self.layout.room_size_m[0] / nx as f32,
                        (y as f32 + 0.5) * self.layout.room_size_m[1] / ny as f32,
                        (z as f32 + 0.5) * self.layout.room_size_m[2] / nz as f32,
                    ];
                    cells.push(SparseCell {
                        x,
                        y,
                        z,
                        probability,
                    });
                    for axis in 0..3 {
                        weighted[axis] += center[axis] * probability;
                        minimum[axis] = minimum[axis].min(center[axis]);
                        maximum[axis] = maximum[axis].max(center[axis]);
                    }
                    weight_sum += probability;
                }
            }
        }
        let centroid_m = (weight_sum > 0.0).then(|| {
            [
                weighted[0] / weight_sum,
                weighted[1] / weight_sum,
                weighted[2] / weight_sum,
            ]
        });
        let extent_m = (!cells.is_empty()).then(|| {
            [
                (maximum[0] - minimum[0]).max(self.layout.room_size_m[0] / nx as f32),
                (maximum[1] - minimum[1]).max(self.layout.room_size_m[1] / ny as f32),
                (maximum[2] - minimum[2]).max(self.layout.room_size_m[2] / nz as f32),
            ]
        });
        let active_links = measurements.len();
        let calibrated_links = self
            .links
            .values()
            .filter(|link| link.baseline.ready())
            .count();
        let confidence = if measurements.is_empty() {
            0.0
        } else {
            let quality = measurements
                .iter()
                .map(|measurement| measurement.quality)
                .sum::<f32>()
                / measurements.len() as f32;
            let coverage = (active_links as f32 / MIN_READY_LINKS as f32).clamp(0.0, 1.0);
            let convergence = if volume.comparison.fallback_used {
                0.55
            } else if volume.comparison.converged {
                1.0
            } else {
                0.75
            };
            (quality * coverage * convergence).clamp(0.0, 1.0)
        };
        let [low, mid, high] = volume.low_mid_high_energy;
        let dominant = if low >= mid && low >= high {
            "low"
        } else if mid >= high {
            "mid"
        } else {
            "high"
        };
        self.occupancy = OccupancyFrame {
            schema: "formmap-occupancy/v1".to_string(),
            frame_id: self.occupancy_frame_id,
            timestamp_ms: now,
            grid_size: self.layout.grid_size,
            room_size_m: self.layout.room_size_m,
            cells,
            centroid_m: centroid_m.or(volume.centroid_m),
            extent_m,
            confidence,
            active_links,
            calibrated_links,
            calibration_id: self.calibration_id(),
            height_layers: HeightLayerSummary {
                low,
                mid,
                high,
                dominant: dominant.to_string(),
            },
            degraded: active_links < MIN_READY_LINKS || volume.comparison.fallback_used,
            solver: volume.comparison,
            note: "RF probability volume; not a camera image or calibrated body mesh"
                .to_string(),
        };
    }

    pub fn snapshot(&self) -> FormMapSnapshot {
        let now = now_ms();
        let mut links = self
            .links
            .values()
            .map(|link| link.snapshot(now))
            .collect::<Vec<_>>();
        links.sort_by_key(|link| (link.node_id, link.station_id));
        let connected_nodes = links
            .iter()
            .filter(|link| link.online)
            .map(|link| link.node_id)
            .collect::<HashSet<_>>()
            .len();
        let connected_stations = self
            .stations
            .values()
            .filter(|station| station.online)
            .count();
        let active_links = links.iter().filter(|link| link.online).count();
        let calibration = self.calibration_status_from_links(&links);
        let sensor_online = self
            .last_csi_ms
            .is_some_and(|last| now.saturating_sub(last) <= DEFAULT_CSI_TIMEOUT_MS);
        let ready = sensor_online
            && calibration.ready_links >= calibration.minimum_ready_links
            && !calibration.recalibration_required;
        let (system_state, reason_code, recovery) = if !sensor_online {
            (
                "OFFLINE",
                "CSI_INPUT_UNAVAILABLE",
                "check node power, target IP and UDP port",
            )
        } else if links.is_empty() {
            (
                "DISCOVERING",
                "NO_LINKS",
                "enroll stations or enable ADR-018 v2 station identity",
            )
        } else if calibration.recalibration_required {
            (
                "DEGRADED",
                "ENVIRONMENT_DRIFT",
                "empty the room and run calibration again",
            )
        } else if calibration.ready_links < calibration.minimum_ready_links {
            (
                "CALIBRATING",
                "BASELINE_NOT_READY",
                "keep devices fixed and room empty until baseline quality passes",
            )
        } else if self.occupancy.degraded {
            (
                "DEGRADED",
                "SOLVER_FALLBACK_OR_LINK_DROPOUT",
                "restore crossing links or inspect link diagnostics",
            )
        } else {
            ("READY", "OK", "none")
        };
        let drift_index = links
            .iter()
            .map(|link| link.baseline.drift_index)
            .fold(0.0f32, f32::max);
        FormMapSnapshot {
            mode: "formmap-live".to_string(),
            system_state: system_state.to_string(),
            reason_code: reason_code.to_string(),
            recovery: recovery.to_string(),
            ready,
            sensor_online,
            csi_frames_received: self.total_frames,
            last_csi_ms: self.last_csi_ms,
            connected_nodes,
            connected_stations,
            active_links,
            drift_index,
            recalibration_required: calibration.recalibration_required,
            simulation_enabled: false,
            camera_enabled: false,
            skeleton_enabled: false,
            calibration,
            nodes: self.node_runtime(&links, now),
            links,
            occupancy: self.occupancy.clone(),
            layout: self.layout.clone(),
        }
    }

    fn calibration_status_from_links(&self, links: &[LinkSnapshot]) -> CalibrationStatus {
        let total_links = links.len();
        let ready_links = links
            .iter()
            .filter(|link| link.baseline.ready && link.quality.final_quality >= self.layout.rti.minimum_link_quality)
            .count();
        let progress = if total_links == 0 {
            0.0
        } else {
            links
                .iter()
                .map(|link| {
                    (link.baseline.frames_collected as f32
                        / link.baseline.target_frames.max(1) as f32)
                        .clamp(0.0, 1.0)
                        * (link.baseline.quality / self.layout.baseline.minimum_quality)
                            .clamp(0.0, 1.0)
                })
                .sum::<f32>()
                / total_links as f32
        };
        let blocked_links = links
            .iter()
            .filter_map(|link| {
                let mut reasons = Vec::new();
                if !link.online {
                    reasons.push("offline".to_string());
                }
                if !link.baseline.ready {
                    reasons.push("baseline_not_ready".to_string());
                }
                if link.baseline.quality < self.layout.baseline.minimum_quality {
                    reasons.push("baseline_quality_low".to_string());
                }
                if link.quality.stationarity_score < 0.75 {
                    reasons.push("station_moved".to_string());
                }
                if link.baseline.recalibration_required {
                    reasons.push("environment_drift".to_string());
                }
                (!reasons.is_empty()).then_some(BlockedLink {
                    link_id: link.link_id,
                    reasons,
                })
            })
            .collect::<Vec<_>>();
        let drift_warning_links = links
            .iter()
            .filter(|link| link.baseline.drift_warning)
            .count();
        let recalibration_required = links
            .iter()
            .any(|link| link.baseline.recalibration_required);
        CalibrationStatus {
            collecting: ready_links < total_links.max(MIN_READY_LINKS),
            target_samples_per_link: self.layout.baseline.target_frames,
            total_links,
            ready_links,
            progress,
            minimum_quality: self.layout.baseline.minimum_quality,
            minimum_ready_links: MIN_READY_LINKS,
            blocked_links,
            drift_warning_links,
            recalibration_required,
            calibration_id: self.calibration_id(),
        }
    }

    fn node_runtime(&self, links: &[LinkSnapshot], now: u64) -> Vec<NodeRuntime> {
        let mut nodes = HashMap::<u8, NodeRuntime>::new();
        for link in links {
            let entry = nodes.entry(link.node_id).or_insert(NodeRuntime {
                node_id: link.node_id,
                last_seen_ms: link.last_seen_ms,
                online: link.online,
                channel: link.channel,
                protocol_version: link.protocol_version,
                firmware_version_code: link.firmware_version_code,
            });
            if link.last_seen_ms > entry.last_seen_ms {
                entry.last_seen_ms = link.last_seen_ms;
                entry.channel = link.channel;
                entry.protocol_version = link.protocol_version;
                entry.firmware_version_code = link.firmware_version_code;
            }
            entry.online = now.saturating_sub(entry.last_seen_ms) <= DEFAULT_CSI_TIMEOUT_MS;
        }
        let mut output = nodes.into_values().collect::<Vec<_>>();
        output.sort_by_key(|node| node.node_id);
        output
    }

    pub fn calibration_id(&self) -> String {
        let mut ids = self
            .links
            .iter()
            .filter_map(|(key, link)| {
                link.baseline
                    .calibration_id()
                    .map(|id| (key.numeric_id(), id.to_string()))
            })
            .collect::<Vec<_>>();
        ids.sort_by_key(|item| item.0);
        if ids.is_empty() {
            return "unavailable".to_string();
        }
        let mut hash = 0xcbf29ce484222325u64;
        for (link_id, id) in ids {
            for byte in link_id.to_le_bytes().into_iter().chain(id.bytes()) {
                hash ^= byte as u64;
                hash = hash.wrapping_mul(0x100000001b3);
            }
        }
        format!("cal-{:016x}", hash)
    }

    pub fn provenance(&self) -> serde_json::Value {
        let snapshot = self.snapshot();
        serde_json::json!({
            "schema": "formmap-provenance/v1",
            "code_version": env!("CARGO_PKG_VERSION"),
            "code_commit": option_env!("GIT_COMMIT_SHA").unwrap_or("unknown"),
            "protocol_versions": snapshot.links.iter().map(|link| link.protocol_version).collect::<HashSet<_>>(),
            "firmware_versions": snapshot.nodes.iter().map(|node| serde_json::json!({
                "node_id": node.node_id,
                "version_code": node.firmware_version_code,
                "channel": node.channel
            })).collect::<Vec<_>>(),
            "layout": snapshot.layout,
            "calibration_id": snapshot.calibration.calibration_id,
            "calibration": snapshot.calibration,
            "solver": self.layout.rti,
            "source_honesty": {
                "camera": false,
                "skeleton": false,
                "automatic_simulation": false
            }
        })
    }

    pub fn topology(&self) -> serde_json::Value {
        let snapshot = self.snapshot();
        serde_json::json!({
            "nodes": snapshot.nodes,
            "node_placements": &self.layout.nodes,
            "station_placements": &self.layout.stations,
            "runtime_stations": self.stations.values().collect::<Vec<_>>(),
            "links": snapshot.links,
            "calibration": snapshot.calibration,
            "solver": snapshot.occupancy.solver,
        })
    }
}

fn mean(values: &[f32]) -> f32 {
    let mut sum = 0.0f32;
    let mut count = 0usize;
    for value in values.iter().copied().filter(|value| value.is_finite()) {
        sum += value;
        count += 1;
    }
    if count == 0 { 0.0 } else { sum / count as f32 }
}

fn normalized_variance(values: &VecDeque<f32>) -> f32 {
    if values.len() < 5 {
        return 0.0;
    }
    let mean = values.iter().sum::<f32>() / values.len() as f32;
    let variance = values
        .iter()
        .map(|value| {
            let delta = *value - mean;
            delta * delta
        })
        .sum::<f32>()
        / values.len() as f32;
    (variance.sqrt() / mean.abs().max(1.0)).clamp(0.0, 1.0)
}

fn hex_hash(hash: &[u8; 8]) -> String {
    hash.iter()
        .map(|byte| format!("{byte:02x}"))
        .collect::<Vec<_>>()
        .join("")
}

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum AgentMessage {
    Hello {
        name: String,
        #[serde(default = "default_platform")]
        platform: String,
        #[serde(default)]
        source_mac_hash: Option<[u8; 8]>,
        #[serde(default)]
        nic: Option<String>,
        #[serde(default)]
        ssid: Option<String>,
        #[serde(default)]
        bssid: Option<String>,
    },
    Probe {
        station_id: u16,
        sequence: u32,
        timestamp_ns: u64,
        #[serde(default = "default_true")]
        stationary: bool,
        #[serde(default)]
        stationarity_score: Option<f32>,
        #[serde(default)]
        rssi_dbm: Option<f32>,
        #[serde(default)]
        link_speed_mbps: Option<f32>,
        #[serde(default)]
        thermal_c: Option<f32>,
    },
}

fn default_platform() -> String {
    std::env::consts::OS.to_string()
}

#[derive(Serialize, Deserialize)]
struct AgentWelcome {
    ok: bool,
    station_id: u16,
    probe_port: u16,
    recommended_rate_hz: u32,
    correlation_window_ms: u64,
}

pub fn start_runtime(
    csi_bind: &str,
    agent_bind: &str,
    discovery_bind: &str,
    layout_path: &str,
) -> Result<SharedState> {
    let state = Arc::new(Mutex::new(FormMapState::load(layout_path)));

    let csi_socket = UdpSocket::bind(csi_bind)?;
    csi_socket.set_read_timeout(Some(Duration::from_secs(1)))?;
    let csi_state = state.clone();
    std::thread::Builder::new()
        .name("formmap-csi".to_string())
        .spawn(move || {
            let mut buffer = [0u8; 8_192];
            loop {
                match csi_socket.recv_from(&mut buffer) {
                    Ok((length, _source)) => {
                        if let Some(frame) = parse_adr018(&buffer[..length]) {
                            if let Ok(mut core) = csi_state.lock() {
                                core.ingest_frame(frame);
                            }
                        }
                    }
                    Err(error)
                        if error.kind() == std::io::ErrorKind::WouldBlock
                            || error.kind() == std::io::ErrorKind::TimedOut => {}
                    Err(error) => eprintln!("CSI socket error: {error}"),
                }
            }
        })?;

    let agent_socket = UdpSocket::bind(agent_bind)?;
    agent_socket.set_read_timeout(Some(Duration::from_secs(1)))?;
    let agent_state = state.clone();
    std::thread::Builder::new()
        .name("formmap-agent-registry".to_string())
        .spawn(move || {
            let mut buffer = [0u8; 4_096];
            loop {
                match agent_socket.recv_from(&mut buffer) {
                    Ok((length, source)) => {
                        let Ok(message) = serde_json::from_slice::<AgentMessage>(&buffer[..length]) else {
                            continue;
                        };
                        match message {
                            AgentMessage::Hello {
                                name,
                                platform,
                                source_mac_hash,
                                nic,
                                ssid,
                                bssid,
                            } => {
                                let station_id = agent_state
                                    .lock()
                                    .map(|mut core| {
                                        core.assign_station(
                                            &name,
                                            &platform,
                                            source,
                                            source_mac_hash,
                                            nic,
                                            ssid,
                                            bssid,
                                        )
                                    })
                                    .unwrap_or(0);
                                let response = AgentWelcome {
                                    ok: station_id != 0,
                                    station_id,
                                    probe_port: agent_socket
                                        .local_addr()
                                        .map(|address| address.port())
                                        .unwrap_or(4100),
                                    recommended_rate_hz: 20,
                                    correlation_window_ms: PROBE_CORRELATION_WINDOW_MS,
                                };
                                if let Ok(data) = serde_json::to_vec(&response) {
                                    let _ = agent_socket.send_to(&data, source);
                                }
                            }
                            AgentMessage::Probe {
                                station_id,
                                sequence,
                                timestamp_ns,
                                stationary,
                                stationarity_score,
                                rssi_dbm,
                                link_speed_mbps,
                                thermal_c,
                            } => {
                                let _ = timestamp_ns;
                                if let Ok(mut core) = agent_state.lock() {
                                    core.record_probe(
                                        station_id,
                                        sequence,
                                        source,
                                        stationary,
                                        stationarity_score,
                                        rssi_dbm,
                                        link_speed_mbps,
                                        thermal_c,
                                    );
                                }
                            }
                        }
                    }
                    Err(error)
                        if error.kind() == std::io::ErrorKind::WouldBlock
                            || error.kind() == std::io::ErrorKind::TimedOut => {}
                    Err(error) => eprintln!("Agent socket error: {error}"),
                }
            }
        })?;

    let discovery_socket = UdpSocket::bind(discovery_bind)?;
    discovery_socket.set_read_timeout(Some(Duration::from_secs(1)))?;
    std::thread::Builder::new()
        .name("formmap-discovery".to_string())
        .spawn(move || {
            let mut buffer = [0u8; 256];
            loop {
                match discovery_socket.recv_from(&mut buffer) {
                    Ok((length, source)) if &buffer[..length] == b"FORMMAP_DISCOVER_V1" => {
                        let response = serde_json::json!({
                            "type": "formmap_core",
                            "version": 2,
                            "agent_port": 4100,
                            "http_port": 9880,
                            "protocol": "ADR-018-v2"
                        });
                        let _ = discovery_socket.send_to(response.to_string().as_bytes(), source);
                    }
                    Ok(_) => {}
                    Err(error)
                        if error.kind() == std::io::ErrorKind::WouldBlock
                            || error.kind() == std::io::ErrorKind::TimedOut => {}
                    Err(error) => eprintln!("Discovery socket error: {error}"),
                }
            }
        })?;

    let solver_state = state.clone();
    std::thread::Builder::new()
        .name("formmap-rti".to_string())
        .spawn(move || loop {
            std::thread::sleep(Duration::from_millis(200));
            if let Ok(mut core) = solver_state.lock() {
                core.solve_occupancy();
            }
        })?;

    Ok(state)
}

fn discover_core() -> Result<SocketAddr> {
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    socket.set_broadcast(true)?;
    socket.set_read_timeout(Some(Duration::from_millis(1_500)))?;
    socket.send_to(b"FORMMAP_DISCOVER_V1", "255.255.255.255:4101")?;
    let mut buffer = [0u8; 1_024];
    let (length, source) = socket.recv_from(&mut buffer)?;
    let value: serde_json::Value = serde_json::from_slice(&buffer[..length])?;
    if value.get("type").and_then(|value| value.as_str()) != Some("formmap_core") {
        return Err(anyhow!("invalid discovery response"));
    }
    let port = value
        .get("agent_port")
        .and_then(|value| value.as_u64())
        .unwrap_or(4100) as u16;
    Ok(SocketAddr::new(source.ip(), port))
}

pub fn run_link_agent(core: Option<&str>, name: &str, rate_hz: u32, count: u64) -> Result<()> {
    let target: SocketAddr = match core {
        Some(value) => value.parse()?,
        None => discover_core()?,
    };
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    socket.set_read_timeout(Some(Duration::from_secs(2)))?;
    let hello = serde_json::json!({
        "type": "hello",
        "name": name,
        "platform": std::env::consts::OS
    });
    socket.send_to(hello.to_string().as_bytes(), target)?;
    let mut response = [0u8; 1_024];
    let (length, _) = socket.recv_from(&mut response)?;
    let welcome: AgentWelcome = serde_json::from_slice(&response[..length])?;
    if !welcome.ok || welcome.station_id == 0 {
        return Err(anyhow!("FormMap Core rejected agent enrollment"));
    }
    println!(
        "FormMap Link Agent enrolled: name={} station_id={} core={}",
        name, welcome.station_id, target
    );

    let interval = Duration::from_secs_f64(1.0 / rate_hz.max(1) as f64);
    let mut sequence = 0u32;
    loop {
        let timestamp_ns = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        let probe = serde_json::json!({
            "type": "probe",
            "station_id": welcome.station_id,
            "sequence": sequence,
            "timestamp_ns": timestamp_ns,
            "stationary": true,
            "stationarity_score": 1.0,
            "padding": "FORMMAP-PROBE-TRAFFIC-000000000000000000000000000000000000000000000000"
        });
        socket.send_to(probe.to_string().as_bytes(), target)?;
        sequence = sequence.wrapping_add(1);
        if count > 0 && sequence as u64 >= count {
            break;
        }
        std::thread::sleep(interval);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::{build_v2_test_frame, parse_adr018};

    fn test_state(target_frames: usize) -> FormMapState {
        let mut layout = RoomLayout::default();
        layout.baseline.target_frames = target_frames;
        layout.baseline.minimum_quality = 0.20;
        layout.rti.minimum_link_quality = 0.05;
        FormMapState::from_layout(
            std::env::temp_dir().join(format!("formmap-test-{}.json", now_ms())),
            layout,
        )
    }

    #[test]
    fn layout_validation_rejects_invalid_grid() {
        let mut layout = RoomLayout::default();
        layout.grid_size = [1, 8, 20];
        assert!(layout.validate().is_err());
    }

    #[test]
    fn v2_frames_create_station_aware_link_and_baseline() {
        let mut state = test_state(40);
        for sequence in 0..50 {
            state.ingest_frame(parse_adr018(&build_v2_test_frame(2, 7, sequence)).unwrap());
        }
        state.solve_occupancy();
        let snapshot = state.snapshot();
        assert_eq!(snapshot.links.len(), 1);
        assert_eq!(snapshot.links[0].node_id, 2);
        assert_eq!(snapshot.links[0].station_id, 7);
        assert_eq!(snapshot.links[0].baseline.frames_collected, 40);
    }

    #[test]
    fn station_movement_invalidates_baseline() {
        let mut state = test_state(40);
        state.stations.insert(
            7,
            StationRuntime {
                station_id: 7,
                name: "test".to_string(),
                platform: "test".to_string(),
                address: "127.0.0.1:1".to_string(),
                last_seen_ms: now_ms(),
                last_sequence: 0,
                online: true,
                stationarity_score: 1.0,
                moved_at_ms: None,
                nic: None,
                ssid: None,
                bssid: None,
                rssi_dbm: None,
                link_speed_mbps: None,
                thermal_c: None,
                source_mac_hash: None,
            },
        );
        for sequence in 0..50 {
            state.ingest_frame(parse_adr018(&build_v2_test_frame(2, 7, sequence)).unwrap());
        }
        state.record_probe(
            7,
            1,
            "127.0.0.1:1".parse().unwrap(),
            false,
            Some(0.0),
            None,
            None,
            None,
        );
        assert!(!state.snapshot().links[0].baseline.ready);
    }

    #[test]
    fn occupancy_dimensions_are_consistent() {
        let state = test_state(40);
        assert_eq!(state.occupancy.grid_size, [20, 10, 20]);
    }
}

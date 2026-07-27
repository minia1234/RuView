//! Axum HTTP/WebSocket server for WiFi FormMap.

use crate::capture::{self, CaptureProvenance, CaptureStatus, SharedCapture};
use crate::formmap::{start_runtime, FormMapSnapshot, RoomLayout, SharedState};
use crate::validation;
use axum::{
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        Path, State,
    },
    http::{HeaderValue, Method, StatusCode},
    response::{Html, IntoResponse, Response},
    routing::{get, post},
    Json, Router,
};
use serde::Deserialize;
use serde_json::{json, Value};
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;
use tower_http::cors::{AllowOrigin, CorsLayer};

#[derive(Clone)]
struct AppState {
    core: SharedState,
    capture: SharedCapture,
    csi_internal: String,
}

type ApiResult = Result<Json<Value>, (StatusCode, Json<Value>)>;

#[derive(Default, Deserialize)]
struct StartCaptureRequest {
    label: Option<String>,
    #[serde(default)]
    consent: bool,
}

#[derive(Deserialize)]
struct ReplayCaptureRequest {
    #[serde(default = "default_replay_speed")]
    speed: f32,
}

fn default_replay_speed() -> f32 {
    1.0
}

pub async fn serve(
    bind: &str,
    csi_bind: &str,
    csi_internal: &str,
    agent_bind: &str,
    discovery_bind: &str,
    layout_path: &str,
    capture_root: &str,
) -> anyhow::Result<()> {
    // The private runtime only receives packets forwarded by the capture proxy
    // or by deterministic replay. ESP32 nodes continue to target csi_bind.
    let core = start_runtime(csi_internal, agent_bind, discovery_bind, layout_path)?;
    let capture = capture::start_capture_proxy(csi_bind, csi_internal, capture_root)?;
    let state = Arc::new(AppState {
        core,
        capture,
        csi_internal: csi_internal.to_string(),
    });

    start_evidence_recorder(state.clone());

    let cors = CorsLayer::new()
        .allow_origin(AllowOrigin::predicate(|origin: &HeaderValue, _| {
            let Ok(value) = origin.to_str() else {
                return false;
            };
            value.starts_with("http://localhost")
                || value.starts_with("http://127.0.0.1")
                || value == "null"
        }))
        .allow_methods([
            Method::GET,
            Method::POST,
            Method::PUT,
            Method::DELETE,
            Method::OPTIONS,
        ])
        .allow_headers([axum::http::header::CONTENT_TYPE]);

    let app = Router::new()
        .route("/", get(index))
        .route("/health", get(health))
        .route("/ws/formmap", get(ws_upgrade))
        // Compatibility endpoints retained for the pre-FormMap viewer.
        .route("/api/status", get(status))
        .route("/api/cloud", get(compat_cloud))
        .route("/api/splats", get(compat_splats))
        .route("/api/v1/status", get(status))
        .route("/api/v1/topology", get(topology))
        .route("/api/v1/links", get(links))
        .route("/api/v1/occupancy", get(occupancy))
        .route("/api/v1/layout", get(get_layout).put(put_layout))
        .route("/api/v1/calibration/status", get(calibration_status))
        .route("/api/v1/calibration/reset", post(reset_calibration))
        // Official FormMap contract.
        .route("/api/v1/formmap/system/status", get(system_status))
        .route("/api/v1/formmap/occupancy", get(official_occupancy))
        .route("/api/v1/formmap/topology", get(topology))
        .route("/api/v1/formmap/links", get(links))
        .route("/api/v1/formmap/layout", get(get_layout).put(put_layout))
        .route(
            "/api/v1/formmap/calibration/status",
            get(calibration_status),
        )
        .route(
            "/api/v1/formmap/calibration/start",
            post(reset_calibration),
        )
        .route("/api/v1/formmap/solver/comparison", get(solver_comparison))
        .route("/api/v1/formmap/drift", get(drift_status))
        .route("/api/v1/formmap/captures", get(list_captures))
        .route("/api/v1/formmap/captures/start", post(start_capture))
        .route("/api/v1/formmap/captures/stop", post(stop_capture))
        .route(
            "/api/v1/formmap/captures/:capture_id",
            get(get_capture).delete(delete_capture),
        )
        .route(
            "/api/v1/formmap/captures/:capture_id/replay",
            post(replay_capture),
        )
        .route(
            "/api/v1/formmap/captures/:capture_id/validate",
            post(validate_capture),
        )
        .route(
            "/api/v1/formmap/validation/suite",
            get(validation_suite),
        )
        .layer(cors)
        .with_state(state);

    println!("╔════════════════════════════════════════════════════╗");
    println!("║  RuView · WiFi FormMap Full Stack                  ║");
    println!("╚════════════════════════════════════════════════════╝");
    println!("  Viewer       : http://{bind}/");
    println!("  CSI public   : {csi_bind} (capture proxy)");
    println!("  CSI internal : {csi_internal} (parser/solver)");
    println!("  Link agents  : {agent_bind}");
    println!("  Discovery    : {discovery_bind}");
    println!("  Layout       : {layout_path}");
    println!("  Captures     : {capture_root}");
    println!("  Camera       : disabled");
    println!("  Simulation   : disabled");
    println!("  Skeleton     : disabled");

    let listener = tokio::net::TcpListener::bind(bind).await?;
    axum::serve(listener, app).await?;
    Ok(())
}

fn start_evidence_recorder(state: Arc<AppState>) {
    tokio::spawn(async move {
        let mut last_frame_id = u64::MAX;
        loop {
            let snapshot = snapshot(&state);
            if snapshot.occupancy.frame_id != last_frame_id {
                last_frame_id = snapshot.occupancy.frame_id;
                let system = system_status_value(&state, &snapshot);
                let occupancy = official_occupancy_value(&state, &snapshot);
                if let Ok(mut capture) = state.capture.lock() {
                    if let Err(error) = capture.record_snapshot(&system, &occupancy) {
                        eprintln!("FormMap evidence record error: {error}");
                    }
                }
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    });
}

fn snapshot(state: &AppState) -> FormMapSnapshot {
    state
        .core
        .lock()
        .expect("FormMap state lock poisoned")
        .snapshot()
}

fn capture_status(state: &AppState) -> CaptureStatus {
    state
        .capture
        .lock()
        .expect("FormMap capture lock poisoned")
        .status()
}

fn source_and_session(capture: &CaptureStatus, sensor_online: bool) -> (&'static str, String) {
    if let Some(replay) = capture.replay.as_ref().filter(|replay| replay.active) {
        return ("replayed", replay.capture_id.clone());
    }
    if let Some(active) = &capture.active {
        return ("measured", active.session_id.clone());
    }
    if sensor_online {
        ("measured", "live-unrecorded".to_string())
    } else {
        ("unavailable", "unavailable".to_string())
    }
}

fn system_status_value(state: &AppState, snapshot: &FormMapSnapshot) -> Value {
    let capture = capture_status(state);
    let (source, session_id) = source_and_session(&capture, snapshot.sensor_online);
    let replaying = capture.replay.as_ref().is_some_and(|replay| replay.active);
    let recording = capture.active.is_some();
    let (system_state, reason_code, recovery) = if replaying {
        (
            "PROCESSING",
            "REPLAY_ACTIVE",
            "wait for replay completion or restart replay",
        )
    } else if recording {
        (
            "CAPTURING",
            "RAW_CAPTURE_ACTIVE",
            "stop capture when the measurement is complete",
        )
    } else {
        (
            snapshot.system_state.as_str(),
            snapshot.reason_code.as_str(),
            snapshot.recovery.as_str(),
        )
    };
    json!({
        "schema": "formmap-system-status/v1",
        "version": 1,
        "timestamp_ms": snapshot.occupancy.timestamp_ms,
        "state": system_state,
        "reason_code": reason_code,
        "recovery": recovery,
        "source": source,
        "session_id": session_id,
        "ready": snapshot.ready,
        "sensor_online": snapshot.sensor_online,
        "csi_frames_received": snapshot.csi_frames_received,
        "connected_nodes": snapshot.connected_nodes,
        "connected_stations": snapshot.connected_stations,
        "active_links": snapshot.active_links,
        "drift_index": snapshot.drift_index,
        "recalibration_required": snapshot.recalibration_required,
        "calibration": snapshot.calibration,
        "recording": recording,
        "replaying": replaying,
        "capture": capture,
        "simulation_enabled": false,
        "camera_enabled": false,
        "skeleton_enabled": false
    })
}

fn official_occupancy_value(state: &AppState, snapshot: &FormMapSnapshot) -> Value {
    let capture = capture_status(state);
    let (source, session_id) = source_and_session(&capture, snapshot.sensor_online);
    let [nx, ny, nz] = snapshot.occupancy.grid_size;
    let [width, height, depth] = snapshot.occupancy.room_size_m;
    let cell_size = [
        width / nx.max(1) as f32,
        height / ny.max(1) as f32,
        depth / nz.max(1) as f32,
    ];
    let cells = snapshot
        .occupancy
        .cells
        .iter()
        .map(|cell| json!([cell.x, cell.y, cell.z, cell.probability]))
        .collect::<Vec<_>>();
    json!({
        "schema": "formmap-occupancy-frame/v1",
        "version": 1,
        "session_id": session_id,
        "frame_id": snapshot.occupancy.frame_id,
        "timestamp": snapshot.occupancy.timestamp_ms,
        "grid_size": snapshot.occupancy.grid_size,
        "cell_size_m": cell_size,
        "room_size_m": snapshot.occupancy.room_size_m,
        "cells": cells,
        "centroid_m": snapshot.occupancy.centroid_m,
        "extent_m": snapshot.occupancy.extent_m,
        "confidence": snapshot.occupancy.confidence,
        "source": source,
        "calibration_id": snapshot.occupancy.calibration_id,
        "active_links": snapshot.occupancy.active_links,
        "calibrated_links": snapshot.occupancy.calibrated_links,
        "height_layers": snapshot.occupancy.height_layers,
        "solver": snapshot.occupancy.solver,
        "degraded": snapshot.occupancy.degraded,
        "note": "RF probability volume; not a camera image, body mesh or identity output"
    })
}

fn api_error(status: StatusCode, error: impl ToString) -> (StatusCode, Json<Value>) {
    (
        status,
        Json(json!({
            "error": {
                "code": status.as_u16(),
                "message": error.to_string()
            }
        })),
    )
}

async fn index() -> Html<&'static str> {
    Html(include_str!("viewer.html"))
}

async fn health(State(state): State<Arc<AppState>>) -> Json<Value> {
    let snapshot = snapshot(&state);
    Json(json!({
        "status": if snapshot.sensor_online { "ok" } else { "degraded" },
        "mode": "formmap-live",
        "system_state": snapshot.system_state,
        "ready": snapshot.ready,
        "simulation_enabled": false,
        "camera_enabled": false,
        "skeleton_enabled": false
    }))
}

async fn status(State(state): State<Arc<AppState>>) -> Json<FormMapSnapshot> {
    Json(snapshot(&state))
}

async fn system_status(State(state): State<Arc<AppState>>) -> Json<Value> {
    let snapshot = snapshot(&state);
    Json(system_status_value(&state, &snapshot))
}

async fn topology(State(state): State<Arc<AppState>>) -> Json<Value> {
    let value = state
        .core
        .lock()
        .expect("FormMap state lock poisoned")
        .topology();
    Json(value)
}

async fn links(State(state): State<Arc<AppState>>) -> Json<Value> {
    let snapshot = snapshot(&state);
    Json(json!({"links": snapshot.links}))
}

async fn occupancy(State(state): State<Arc<AppState>>) -> Json<Value> {
    let snapshot = snapshot(&state);
    Json(serde_json::to_value(snapshot.occupancy).unwrap_or_else(|_| json!({})))
}

async fn official_occupancy(State(state): State<Arc<AppState>>) -> Json<Value> {
    let snapshot = snapshot(&state);
    Json(official_occupancy_value(&state, &snapshot))
}

async fn calibration_status(State(state): State<Arc<AppState>>) -> Json<Value> {
    let snapshot = snapshot(&state);
    Json(serde_json::to_value(snapshot.calibration).unwrap_or_else(|_| json!({})))
}

async fn reset_calibration(State(state): State<Arc<AppState>>) -> Json<Value> {
    state
        .core
        .lock()
        .expect("FormMap state lock poisoned")
        .reset_calibration();
    Json(json!({
        "ok": true,
        "message": "empty-room robust per-subcarrier baseline collection restarted"
    }))
}

async fn solver_comparison(State(state): State<Arc<AppState>>) -> Json<Value> {
    let snapshot = snapshot(&state);
    Json(json!({
        "solver": snapshot.occupancy.solver,
        "degraded": snapshot.occupancy.degraded,
        "height_layers": snapshot.occupancy.height_layers
    }))
}

async fn drift_status(State(state): State<Arc<AppState>>) -> Json<Value> {
    let snapshot = snapshot(&state);
    let links = snapshot
        .links
        .iter()
        .map(|link| {
            json!({
                "link_id": link.link_id,
                "drift_index": link.baseline.drift_index,
                "warning": link.baseline.drift_warning,
                "recalibration_required": link.baseline.recalibration_required,
                "invalidation_reason": link.baseline.invalidation_reason
            })
        })
        .collect::<Vec<_>>();
    Json(json!({
        "drift_index": snapshot.drift_index,
        "recalibration_required": snapshot.recalibration_required,
        "links": links
    }))
}

async fn get_layout(State(state): State<Arc<AppState>>) -> Json<RoomLayout> {
    let layout = state
        .core
        .lock()
        .expect("FormMap state lock poisoned")
        .layout
        .clone();
    Json(layout)
}

async fn put_layout(
    State(state): State<Arc<AppState>>,
    Json(layout): Json<RoomLayout>,
) -> ApiResult {
    state
        .core
        .lock()
        .map_err(|_| api_error(StatusCode::INTERNAL_SERVER_ERROR, "FormMap state lock poisoned"))?
        .replace_layout(layout)
        .map_err(|error| api_error(StatusCode::BAD_REQUEST, error))?;
    Ok(Json(json!({
        "ok": true,
        "message": "layout stored; affected link baselines were invalidated"
    })))
}

async fn list_captures(State(state): State<Arc<AppState>>) -> ApiResult {
    let captures = state
        .capture
        .lock()
        .map_err(|_| api_error(StatusCode::INTERNAL_SERVER_ERROR, "capture lock poisoned"))?
        .list_captures()
        .map_err(|error| api_error(StatusCode::INTERNAL_SERVER_ERROR, error))?;
    Ok(Json(json!({"captures": captures})))
}

async fn get_capture(
    Path(capture_id): Path<String>,
    State(state): State<Arc<AppState>>,
) -> ApiResult {
    let capture = state
        .capture
        .lock()
        .map_err(|_| api_error(StatusCode::INTERNAL_SERVER_ERROR, "capture lock poisoned"))?
        .get_capture(&capture_id)
        .map_err(|error| api_error(StatusCode::NOT_FOUND, error))?;
    Ok(Json(json!({"capture": capture})))
}

async fn start_capture(
    State(state): State<Arc<AppState>>,
    Json(request): Json<StartCaptureRequest>,
) -> ApiResult {
    let provenance = state
        .core
        .lock()
        .map_err(|_| api_error(StatusCode::INTERNAL_SERVER_ERROR, "FormMap state lock poisoned"))?
        .provenance();
    let manifest = state
        .capture
        .lock()
        .map_err(|_| api_error(StatusCode::INTERNAL_SERVER_ERROR, "capture lock poisoned"))?
        .start_capture(
            request.label.as_deref(),
            CaptureProvenance::from_value(provenance),
            request.consent,
        )
        .map_err(|error| api_error(StatusCode::CONFLICT, error))?;
    Ok(Json(json!({"capture": manifest})))
}

async fn stop_capture(State(state): State<Arc<AppState>>) -> ApiResult {
    let manifest = state
        .capture
        .lock()
        .map_err(|_| api_error(StatusCode::INTERNAL_SERVER_ERROR, "capture lock poisoned"))?
        .stop_capture()
        .map_err(|error| api_error(StatusCode::CONFLICT, error))?;
    Ok(Json(json!({"capture": manifest})))
}

async fn delete_capture(
    Path(capture_id): Path<String>,
    State(state): State<Arc<AppState>>,
) -> ApiResult {
    state
        .capture
        .lock()
        .map_err(|_| api_error(StatusCode::INTERNAL_SERVER_ERROR, "capture lock poisoned"))?
        .delete_capture(&capture_id)
        .map_err(|error| api_error(StatusCode::CONFLICT, error))?;
    Ok(Json(json!({"ok": true, "capture_id": capture_id})))
}

async fn replay_capture(
    Path(capture_id): Path<String>,
    State(state): State<Arc<AppState>>,
    Json(request): Json<ReplayCaptureRequest>,
) -> ApiResult {
    let replay = capture::spawn_replay(
        state.capture.clone(),
        &capture_id,
        &state.csi_internal,
        request.speed,
    )
    .map_err(|error| api_error(StatusCode::BAD_REQUEST, error))?;
    Ok(Json(json!({"replay": replay})))
}

async fn validate_capture(
    Path(capture_id): Path<String>,
    State(state): State<Arc<AppState>>,
) -> ApiResult {
    let root = PathBuf::from(capture_status(&state).root);
    let report = validation::run_hardware_validation(root, &capture_id, None, None)
        .map_err(|error| api_error(StatusCode::BAD_REQUEST, error))?;
    Ok(Json(serde_json::to_value(report).unwrap_or_else(|_| json!({}))))
}

async fn validation_suite() -> Json<Value> {
    Json(validation::suite_definition())
}

async fn compat_cloud(State(state): State<Arc<AppState>>) -> Json<Value> {
    let snapshot = snapshot(&state);
    let [nx, ny, nz] = snapshot.occupancy.grid_size;
    let [width, height, depth] = snapshot.occupancy.room_size_m;
    let points = snapshot
        .occupancy
        .cells
        .iter()
        .map(|cell| {
            json!({
                "x": (cell.x as f32 + 0.5) * width / nx as f32,
                "y": (cell.y as f32 + 0.5) * height / ny as f32,
                "z": (cell.z as f32 + 0.5) * depth / nz as f32,
                "intensity": cell.probability
            })
        })
        .collect::<Vec<_>>();
    Json(json!({
        "live": snapshot.sensor_online,
        "source": "wifi-formmap-csi",
        "points": points.len(),
        "cloud": points,
        "pipeline": snapshot
    }))
}

async fn compat_splats(State(state): State<Arc<AppState>>) -> Json<Value> {
    let snapshot = snapshot(&state);
    let [nx, ny, nz] = snapshot.occupancy.grid_size;
    let [width, height, depth] = snapshot.occupancy.room_size_m;
    let splats = snapshot
        .occupancy
        .cells
        .iter()
        .map(|cell| {
            let probability = cell.probability;
            json!({
                "center": [
                    (cell.x as f32 + 0.5) * width / nx as f32,
                    (cell.y as f32 + 0.5) * height / ny as f32,
                    (cell.z as f32 + 0.5) * depth / nz as f32
                ],
                "color": [probability, 0.35 + probability * 0.45, 1.0 - probability * 0.65],
                "opacity": probability,
                "scale": [
                    width / nx as f32 * 0.45,
                    height / ny as f32 * 0.45,
                    depth / nz as f32 * 0.45
                ]
            })
        })
        .collect::<Vec<_>>();
    Json(json!({
        "live": snapshot.sensor_online,
        "source": "wifi-formmap-csi",
        "count": splats.len(),
        "splats": splats,
        "pipeline": snapshot
    }))
}

async fn ws_upgrade(
    ws: WebSocketUpgrade,
    State(state): State<Arc<AppState>>,
) -> Response {
    ws.on_upgrade(move |socket| websocket_loop(socket, state))
        .into_response()
}

async fn websocket_loop(mut socket: WebSocket, state: Arc<AppState>) {
    loop {
        let snapshot = snapshot(&state);
        let payload = json!({
            "system": system_status_value(&state, &snapshot),
            "occupancy": official_occupancy_value(&state, &snapshot),
            "links": snapshot.links,
            "nodes": snapshot.nodes,
            "layout": snapshot.layout,
            "calibration": snapshot.calibration
        });
        let Ok(text) = serde_json::to_string(&payload) else {
            return;
        };
        if socket.send(Message::Text(text)).await.is_err() {
            return;
        }
        tokio::time::sleep(Duration::from_millis(250)).await;
    }
}

//! Honest live-mode HTTP server for real ESP32 CSI input.
//!
//! This server deliberately does not create camera, face-mesh, skeleton, or
//! synthetic point-cloud output. When no ADR-018 CSI frames are arriving the
//! API reports an offline sensor and the viewer remains empty.

use crate::csi_pipeline;
use axum::{
    extract::State,
    http::{HeaderValue, Method},
    response::Html,
    routing::get,
    Json, Router,
};
use std::sync::{Arc, Mutex};
use tower_http::cors::{AllowOrigin, CorsLayer};

struct AppState {
    latest_pipeline: Mutex<Option<csi_pipeline::PipelineOutput>>,
    publish_frame: Mutex<u64>,
}

/// Start the CSI-only viewer server.
///
/// The HTTP server defaults to loopback. ESP32 nodes send ADR-018 frames to
/// UDP 3333. No simulation or camera fallback is started by this function.
pub async fn serve(bind: &str, _brain: Option<&str>) -> anyhow::Result<()> {
    let csi_pipeline_state = csi_pipeline::start_pipeline("0.0.0.0:3333");
    eprintln!("  CSI input: UDP 3333 (ADR-018 binary frames)");
    eprintln!("  Camera: DISABLED");
    eprintln!("  Simulation: DISABLED");
    eprintln!("  Skeleton output: DISABLED");

    let state = Arc::new(AppState {
        latest_pipeline: Mutex::new(None),
        publish_frame: Mutex::new(0),
    });

    // Publish snapshots from the real CSI pipeline only. The pipeline still
    // computes legacy fields internally for compatibility, but skeleton data is
    // stripped at this trust boundary and never exposed to the live-only UI.
    let bg = state.clone();
    let bg_csi = csi_pipeline_state.clone();
    tokio::spawn(async move {
        loop {
            tokio::time::sleep(std::time::Duration::from_millis(250)).await;

            let mut out = csi_pipeline::get_pipeline_output(&bg_csi);
            out.skeleton = None;

            *bg.latest_pipeline.lock().unwrap() = Some(out);
            let mut frame = bg.publish_frame.lock().unwrap();
            *frame += 1;
        }
    });

    let cors = CorsLayer::new()
        .allow_origin(AllowOrigin::predicate(|origin: &HeaderValue, _req| {
            let s = match origin.to_str() {
                Ok(v) => v,
                Err(_) => return false,
            };
            s == "https://ruvnet.github.io"
                || s.starts_with("http://localhost")
                || s.starts_with("http://127.0.0.1")
                || s == "null"
        }))
        .allow_methods([Method::GET, Method::OPTIONS])
        .allow_headers([axum::http::header::CONTENT_TYPE]);

    let app = Router::new()
        .route("/", get(index))
        .route("/api/cloud", get(api_cloud))
        .route("/api/splats", get(api_splats))
        .route("/api/status", get(api_status))
        .route("/health", get(api_health))
        .layer(cors)
        .with_state(state);

    println!("╔══════════════════════════════════════════════╗");
    println!("║  RuView · FormMap Honest Live Mode           ║");
    println!("╚══════════════════════════════════════════════╝");
    println!("  Viewer: http://{bind}/");
    println!("  Waiting for real ESP32 CSI on UDP 3333");

    if bind.starts_with("0.0.0.0") || bind.starts_with("::") {
        eprintln!(
            "  WARNING: bound to {bind}; live CSI-derived occupancy is exposed to the LAN."
        );
    }

    let listener = tokio::net::TcpListener::bind(bind).await?;
    axum::serve(listener, app).await?;
    Ok(())
}

/// Compatibility endpoint. Honest live mode does not expose camera or
/// synthetic point-cloud points.
async fn api_cloud(State(state): State<Arc<AppState>>) -> Json<serde_json::Value> {
    let frame = *state.publish_frame.lock().unwrap();
    let pipeline = state.latest_pipeline.lock().unwrap();
    let total_frames = pipeline.as_ref().map_or(0, |p| p.total_frames);
    Json(serde_json::json!({
        "points": 0,
        "bounds_min": [0.0, 0.0, 0.0],
        "bounds_max": [0.0, 0.0, 0.0],
        "live": total_frames > 0,
        "source": "esp32-csi-only",
        "frame": frame,
        "pipeline": &*pipeline,
        "cloud": []
    }))
}

/// Compatibility endpoint. The viewer uses the real CSI occupancy tensor in
/// `pipeline.occupancy`; no fabricated Gaussian splats are returned.
async fn api_splats(State(state): State<Arc<AppState>>) -> Json<serde_json::Value> {
    let frame = *state.publish_frame.lock().unwrap();
    let pipeline = state.latest_pipeline.lock().unwrap();
    let total_frames = pipeline.as_ref().map_or(0, |p| p.total_frames);
    Json(serde_json::json!({
        "splats": [],
        "count": 0,
        "live": total_frames > 0,
        "source": "esp32-csi-only",
        "frame": frame,
        "pipeline": &*pipeline,
        "timestamp": chrono::Utc::now().timestamp_millis()
    }))
}

async fn api_status(State(state): State<Arc<AppState>>) -> Json<serde_json::Value> {
    let publish_frames = *state.publish_frame.lock().unwrap();
    let pipeline = state.latest_pipeline.lock().unwrap();
    let total_frames = pipeline.as_ref().map_or(0, |p| p.total_frames);
    let node_count = pipeline.as_ref().map_or(0, |p| p.num_nodes);
    let sensor_online = total_frames > 0 && node_count > 0;

    Json(serde_json::json!({
        "status": if sensor_online { "sensor_online" } else { "sensor_offline" },
        "version": env!("CARGO_PKG_VERSION"),
        "mode": "honest-live",
        "source": "esp32",
        "require_live": true,
        "simulation_enabled": false,
        "camera_enabled": false,
        "skeleton_enabled": false,
        "sensor_online": sensor_online,
        "connected_nodes": node_count,
        "csi_frames_received": total_frames,
        "publish_frames": publish_frames,
        "csi_pipeline": "listening (UDP:3333)",
        "pipeline": &*pipeline
    }))
}

async fn api_health() -> Json<serde_json::Value> {
    Json(serde_json::json!({
        "status": "ok",
        "mode": "honest-live",
        "simulation_enabled": false
    }))
}

static VIEWER_HTML: &str = include_str!("viewer.html");

async fn index() -> Html<&'static str> {
    Html(VIEWER_HTML)
}

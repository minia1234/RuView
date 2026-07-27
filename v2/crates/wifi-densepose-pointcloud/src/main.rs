//! WiFi FormMap — real CSI-only occupancy mapping.
//!
//! Commands:
//!   ruview-pointcloud serve
//!   ruview-pointcloud agent --name desk-left
//!   ruview-pointcloud csi-test --station-id 1
//!   ruview-pointcloud validate --capture-id formmap-...

mod agent;
mod baseline;
mod capture;
mod formmap;
mod parser;
mod rti;
mod stream;
mod synchronizer;
mod validation;

use anyhow::Result;
use clap::{Parser, Subcommand};
use std::path::PathBuf;

const VERSION: &str = env!("CARGO_PKG_VERSION");

#[derive(Parser)]
#[command(name = "ruview-pointcloud", version = VERSION)]
#[command(about = "Station-aware Wi-Fi CSI occupancy mapping")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Start the real CSI server, capture proxy, calibration and volume viewer.
    Serve {
        #[arg(long, default_value = "127.0.0.1:9880")]
        bind: String,
        /// Public UDP endpoint configured on ESP32 nodes.
        #[arg(long, default_value = "0.0.0.0:3333")]
        csi_bind: String,
        /// Private ingest endpoint used by capture/replay. Keep this loopback-only.
        #[arg(long, default_value = "127.0.0.1:3334")]
        csi_internal: String,
        #[arg(long, default_value = "0.0.0.0:4100")]
        agent_bind: String,
        #[arg(long, default_value = "0.0.0.0:4101")]
        discovery_bind: String,
        #[arg(long, default_value = "formmap-layout.json")]
        layout: String,
        #[arg(long, default_value = "formmap-captures")]
        capture_root: String,
    },
    /// Run a Windows/Linux Link Agent. It auto-discovers the Core unless --core is set.
    Agent {
        #[arg(long)]
        core: Option<String>,
        #[arg(long, default_value = "formmap-agent")]
        name: String,
        #[arg(long, default_value = "20")]
        rate_hz: u32,
        /// 0 means run until interrupted.
        #[arg(long, default_value = "0")]
        count: u64,
        /// Wi-Fi NIC MAC. When omitted, Windows/Linux auto-detection is attempted.
        #[arg(long)]
        source_mac: Option<String>,
        /// Precomputed 16-hex-character FormMap salted MAC hash.
        #[arg(long)]
        source_mac_hash: Option<String>,
        /// Must match CONFIG_FORMMAP_MAC_HASH_SALT in ESP32 firmware.
        #[arg(long)]
        mac_hash_salt: Option<String>,
    },
    /// Send explicit ADR-018 v2 test frames to the public CSI input.
    CsiTest {
        #[arg(long, default_value = "127.0.0.1:3333")]
        target: String,
        #[arg(long, default_value = "800")]
        count: usize,
        #[arg(long, default_value = "1")]
        node_id: u8,
        #[arg(long, default_value = "1")]
        station_id: u16,
    },
    /// Evaluate a recorded hardware session against T-01 through T-08 labels.
    Validate {
        #[arg(long, default_value = "formmap-captures")]
        capture_root: PathBuf,
        #[arg(long)]
        capture_id: String,
        /// Optional JSONL labels. Defaults to <capture>/validation-labels.jsonl.
        #[arg(long)]
        labels: Option<PathBuf>,
        /// Optional report output. Defaults to <capture>/hardware-validation-report.json.
        #[arg(long)]
        output: Option<PathBuf>,
    },
    /// Print the exact T-01 through T-08 validation contract as JSON.
    ValidationSuite,
}

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Commands::Serve {
            bind,
            csi_bind,
            csi_internal,
            agent_bind,
            discovery_bind,
            layout,
            capture_root,
        } => {
            stream::serve(
                &bind,
                &csi_bind,
                &csi_internal,
                &agent_bind,
                &discovery_bind,
                &layout,
                &capture_root,
            )
            .await?;
        }
        Commands::Agent {
            core,
            name,
            rate_hz,
            count,
            source_mac,
            source_mac_hash,
            mac_hash_salt,
        } => {
            agent::run_link_agent(
                core.as_deref(),
                &name,
                rate_hz,
                count,
                source_mac.as_deref(),
                source_mac_hash.as_deref(),
                mac_hash_salt.as_deref(),
            )?;
        }
        Commands::CsiTest {
            target,
            count,
            node_id,
            station_id,
        } => {
            parser::send_test_frames(&target, count, node_id, station_id)?;
        }
        Commands::Validate {
            capture_root,
            capture_id,
            labels,
            output,
        } => {
            let report = validation::run_hardware_validation(
                capture_root,
                &capture_id,
                labels.as_deref(),
                output.as_deref(),
            )?;
            println!("{}", serde_json::to_string_pretty(&report)?);
            if report.failed > 0 {
                std::process::exit(2);
            }
        }
        Commands::ValidationSuite => {
            println!(
                "{}",
                serde_json::to_string_pretty(&validation::suite_definition())?
            );
        }
    }
    Ok(())
}

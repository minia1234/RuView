//! Windows/Linux FormMap Link Agent with deterministic firmware station identity.

use anyhow::{anyhow, Context, Result};
use serde::{Deserialize, Serialize};
use std::fs;
use std::net::{SocketAddr, UdpSocket};
use std::process::Command;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

const DEFAULT_MAC_HASH_SALT: &str = "ruview-formmap-local-salt";
const DISCOVERY_PORT: u16 = 4101;
const DISCOVERY_MESSAGE: &[u8] = b"FORMMAP_DISCOVER_V1";

#[derive(Debug, Serialize)]
pub struct AgentIdentity {
    pub source_mac: Option<String>,
    pub source_mac_hash: Option<[u8; 8]>,
    pub station_id: Option<u16>,
    pub source: String,
}

#[derive(Deserialize)]
struct AgentWelcome {
    ok: bool,
    station_id: u16,
    probe_port: u16,
    recommended_rate_hz: u32,
    correlation_window_ms: u64,
}

pub fn run_link_agent(
    core: Option<&str>,
    name: &str,
    requested_rate_hz: u32,
    count: u64,
    source_mac: Option<&str>,
    source_mac_hash_hex: Option<&str>,
    mac_hash_salt: Option<&str>,
) -> Result<()> {
    let target: SocketAddr = match core {
        Some(value) => value.parse()?,
        None => discover_core()?,
    };
    let identity = resolve_identity(source_mac, source_mac_hash_hex, mac_hash_salt)?;
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    socket.set_read_timeout(Some(Duration::from_secs(2)))?;
    let hello = serde_json::json!({
        "type": "hello",
        "name": name,
        "platform": std::env::consts::OS,
        "source_mac_hash": identity.source_mac_hash,
        "nic": detect_wifi_interface_name(),
        "ssid": ValueOrNull::null(),
        "bssid": ValueOrNull::null()
    });
    socket.send_to(hello.to_string().as_bytes(), target)?;
    let mut response = [0u8; 1_024];
    let (length, _) = socket.recv_from(&mut response)?;
    let welcome: AgentWelcome = serde_json::from_slice(&response[..length])?;
    if !welcome.ok || welcome.station_id == 0 {
        return Err(anyhow!("FormMap Core rejected agent enrollment"));
    }
    if let Some(expected) = identity.station_id {
        if expected != welcome.station_id {
            return Err(anyhow!(
                "station identity mismatch: firmware-derived={} server-assigned={}; check MAC/salt",
                expected,
                welcome.station_id
            ));
        }
    }

    let effective_rate = requested_rate_hz
        .max(1)
        .min(welcome.recommended_rate_hz.max(1).max(requested_rate_hz));
    println!(
        "FormMap Link Agent enrolled: name={} station_id={} identity={} core={} rate={}Hz window={}ms port={}",
        name,
        welcome.station_id,
        identity.source,
        target,
        effective_rate,
        welcome.correlation_window_ms,
        welcome.probe_port
    );
    if identity.source_mac_hash.is_none() {
        eprintln!(
            "warning: no Wi-Fi MAC identity was available; ADR-018 v2 firmware station identity may not match this Agent"
        );
    }

    let interval = Duration::from_secs_f64(1.0 / effective_rate as f64);
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

pub fn resolve_identity(
    source_mac: Option<&str>,
    source_mac_hash_hex: Option<&str>,
    mac_hash_salt: Option<&str>,
) -> Result<AgentIdentity> {
    if let Some(hash) = source_mac_hash_hex {
        let parsed = parse_hash(hash)?;
        return Ok(AgentIdentity {
            source_mac: None,
            source_mac_hash: Some(parsed),
            station_id: Some(stable_station_id(&parsed)),
            source: "explicit-hash".to_string(),
        });
    }

    let detected = match source_mac {
        Some(value) => Some((value.to_string(), "explicit-mac".to_string())),
        None => detect_wifi_mac().map(|value| (value, "auto-detected-mac".to_string())),
    };
    let Some((mac, source)) = detected else {
        return Ok(AgentIdentity {
            source_mac: None,
            source_mac_hash: None,
            station_id: None,
            source: "unavailable".to_string(),
        });
    };
    let bytes = parse_mac(&mac)?;
    let hash = salted_mac_hash(&bytes, mac_hash_salt.unwrap_or(DEFAULT_MAC_HASH_SALT));
    Ok(AgentIdentity {
        source_mac: Some(mac),
        source_mac_hash: Some(hash),
        station_id: Some(stable_station_id(&hash)),
        source,
    })
}

fn discover_core() -> Result<SocketAddr> {
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    socket.set_broadcast(true)?;
    socket.set_read_timeout(Some(Duration::from_millis(1_500)))?;
    socket.send_to(DISCOVERY_MESSAGE, ("255.255.255.255", DISCOVERY_PORT))?;
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

fn detect_wifi_mac() -> Option<String> {
    #[cfg(target_os = "linux")]
    {
        let entries = fs::read_dir("/sys/class/net").ok()?;
        for entry in entries.flatten() {
            let path = entry.path();
            if !path.join("wireless").exists() {
                continue;
            }
            let value = fs::read_to_string(path.join("address")).ok()?;
            let value = value.trim().to_string();
            if parse_mac(&value).is_ok() {
                return Some(value);
            }
        }
    }
    #[cfg(target_os = "windows")]
    {
        let script = "Get-NetAdapter -Physical | Where-Object {$_.Status -eq 'Up' -and ($_.NdisPhysicalMedium -match '802.11|Wireless')} | Select-Object -First 1 -ExpandProperty MacAddress";
        let output = Command::new("powershell")
            .args(["-NoProfile", "-Command", script])
            .output()
            .ok()?;
        if output.status.success() {
            let value = String::from_utf8_lossy(&output.stdout).trim().to_string();
            if parse_mac(&value).is_ok() {
                return Some(value);
            }
        }
    }
    None
}

fn detect_wifi_interface_name() -> Option<String> {
    #[cfg(target_os = "linux")]
    {
        let entries = fs::read_dir("/sys/class/net").ok()?;
        for entry in entries.flatten() {
            if entry.path().join("wireless").exists() {
                return entry.file_name().into_string().ok();
            }
        }
    }
    #[cfg(target_os = "windows")]
    {
        let script = "Get-NetAdapter -Physical | Where-Object {$_.Status -eq 'Up' -and ($_.NdisPhysicalMedium -match '802.11|Wireless')} | Select-Object -First 1 -ExpandProperty Name";
        let output = Command::new("powershell")
            .args(["-NoProfile", "-Command", script])
            .output()
            .ok()?;
        if output.status.success() {
            let value = String::from_utf8_lossy(&output.stdout).trim().to_string();
            if !value.is_empty() {
                return Some(value);
            }
        }
    }
    None
}

fn parse_mac(value: &str) -> Result<[u8; 6]> {
    let compact = value
        .chars()
        .filter(|character| character.is_ascii_hexdigit())
        .collect::<String>();
    if compact.len() != 12 {
        return Err(anyhow!("MAC address must contain 12 hexadecimal digits"));
    }
    let mut output = [0u8; 6];
    for index in 0..6 {
        output[index] = u8::from_str_radix(&compact[index * 2..index * 2 + 2], 16)
            .context("invalid MAC address")?;
    }
    Ok(output)
}

fn parse_hash(value: &str) -> Result<[u8; 8]> {
    let compact = value
        .chars()
        .filter(|character| character.is_ascii_hexdigit())
        .collect::<String>();
    if compact.len() != 16 {
        return Err(anyhow!("source MAC hash must contain 16 hexadecimal digits"));
    }
    let mut output = [0u8; 8];
    for index in 0..8 {
        output[index] = u8::from_str_radix(&compact[index * 2..index * 2 + 2], 16)
            .context("invalid source MAC hash")?;
    }
    Ok(output)
}

fn salted_mac_hash(mac: &[u8; 6], salt: &str) -> [u8; 8] {
    let mut hash = 1469598103934665603u64;
    for byte in salt.bytes().chain(mac.iter().copied()) {
        hash ^= byte as u64;
        hash = hash.wrapping_mul(1099511628211u64);
    }
    hash.to_le_bytes()
}

fn stable_station_id(hash: &[u8; 8]) -> u16 {
    let mut value = 0x811cu16;
    for byte in hash {
        value ^= *byte as u16;
        value = value.wrapping_mul(0x0193);
    }
    value.max(1)
}

struct ValueOrNull;
impl ValueOrNull {
    fn null() -> serde_json::Value {
        serde_json::Value::Null
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn firmware_and_agent_hash_contract_is_stable() {
        let mac = parse_mac("00:11:22:33:44:55").unwrap();
        let left = salted_mac_hash(&mac, DEFAULT_MAC_HASH_SALT);
        let right = salted_mac_hash(&mac, DEFAULT_MAC_HASH_SALT);
        assert_eq!(left, right);
        assert_ne!(stable_station_id(&left), 0);
    }

    #[test]
    fn explicit_hash_parses() {
        let identity = resolve_identity(None, Some("0102030405060708"), None).unwrap();
        assert_eq!(identity.source_mac_hash, Some([1, 2, 3, 4, 5, 6, 7, 8]));
    }
}

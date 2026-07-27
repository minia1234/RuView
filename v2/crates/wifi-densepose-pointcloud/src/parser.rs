//! ADR-018 v1/v6 and FormMap ADR-018 v2 CSI parser.
//!
//! v1/v6 are accepted exactly as emitted by existing RuView ESP32 firmware.
//! v2 adds deterministic station/link identity, firmware provenance, timestamps,
//! a salted source MAC hash and payload CRC16.

use serde::Serialize;
use std::net::UdpSocket;
use std::time::{SystemTime, UNIX_EPOCH};

pub const MAGIC_V1: u32 = 0xC511_0001;
pub const MAGIC_V2: u32 = 0xC511_0002;
pub const MAGIC_V6: u32 = 0xC511_0006;
pub const HEADER_V1: usize = 20;
pub const HEADER_V2: usize = 48;

#[derive(Clone, Debug, Serialize)]
pub struct CsiFrame {
    pub protocol_version: u8,
    pub node_id: u8,
    pub station_id: u16,
    pub link_id: u32,
    pub n_antennas: u8,
    pub n_subcarriers: u16,
    pub frequency_mhz: u32,
    pub channel: u8,
    pub sequence: u32,
    pub agent_sequence: u32,
    pub rssi: i8,
    pub noise_floor: i8,
    pub flags: u16,
    pub firmware_version_code: u16,
    pub timestamp_us: u64,
    pub received_at_ms: u64,
    pub source_mac_hash: [u8; 8],
    pub payload_crc16: u16,
    pub iq_data: Vec<i8>,
    pub amplitudes: Vec<f32>,
    pub phases: Vec<f32>,
}

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

fn channel_from_frequency(frequency_mhz: u32) -> u8 {
    if frequency_mhz == 2484 {
        14
    } else if (2412..=2472).contains(&frequency_mhz) {
        (((frequency_mhz - 2412) / 5) + 1) as u8
    } else if (5000..=5900).contains(&frequency_mhz) {
        ((frequency_mhz - 5000) / 5) as u8
    } else {
        0
    }
}

fn frequency_from_channel(channel: u8) -> u32 {
    if channel == 14 {
        2484
    } else if (1..=13).contains(&channel) {
        2407 + channel as u32 * 5
    } else if channel >= 36 {
        5000 + channel as u32 * 5
    } else {
        0
    }
}

fn calculate_features(iq_data: &[i8], n_subcarriers: usize) -> (Vec<f32>, Vec<f32>) {
    let mut amplitudes = Vec::with_capacity(n_subcarriers);
    let mut phases = Vec::with_capacity(n_subcarriers);
    for index in 0..n_subcarriers {
        let offset = index * 2;
        if offset + 1 >= iq_data.len() {
            break;
        }
        let real = iq_data[offset] as f32;
        let imaginary = iq_data[offset + 1] as f32;
        amplitudes.push((real * real + imaginary * imaginary).sqrt());
        phases.push(imaginary.atan2(real));
    }
    (amplitudes, phases)
}

pub fn parse_adr018(data: &[u8]) -> Option<CsiFrame> {
    if data.len() < 4 {
        return None;
    }
    let magic = u32::from_le_bytes(data[0..4].try_into().ok()?);
    match magic {
        MAGIC_V1 | MAGIC_V6 => parse_v1(data, magic),
        MAGIC_V2 => parse_v2(data),
        _ => None,
    }
}

fn parse_v1(data: &[u8], magic: u32) -> Option<CsiFrame> {
    if data.len() < HEADER_V1 {
        return None;
    }
    let node_id = data[4];
    let n_antennas = data[5].max(1);
    let n_subcarriers = u16::from_le_bytes(data[6..8].try_into().ok()?);
    let frequency_mhz = u32::from_le_bytes(data[8..12].try_into().ok()?);
    let sequence = u32::from_le_bytes(data[12..16].try_into().ok()?);
    let rssi = data[16] as i8;
    let noise_floor = data[17] as i8;
    let flags = u16::from_le_bytes([data[18], data[19]]);
    let iq_length = n_subcarriers as usize * 2 * n_antennas as usize;
    if n_subcarriers == 0 || iq_length > 8_192 || data.len() < HEADER_V1 + iq_length {
        return None;
    }
    let iq_data = data[HEADER_V1..HEADER_V1 + iq_length]
        .iter()
        .map(|value| *value as i8)
        .collect::<Vec<_>>();
    let (amplitudes, phases) = calculate_features(&iq_data, n_subcarriers as usize);
    Some(CsiFrame {
        protocol_version: if magic == MAGIC_V6 { 6 } else { 1 },
        node_id,
        station_id: 0,
        link_id: node_id as u32,
        n_antennas,
        n_subcarriers,
        frequency_mhz,
        channel: channel_from_frequency(frequency_mhz),
        sequence,
        agent_sequence: 0,
        rssi,
        noise_floor,
        flags,
        firmware_version_code: 0,
        timestamp_us: 0,
        received_at_ms: now_ms(),
        source_mac_hash: [0; 8],
        payload_crc16: 0,
        iq_data,
        amplitudes,
        phases,
    })
}

fn parse_v2(data: &[u8]) -> Option<CsiFrame> {
    if data.len() < HEADER_V2 || data[4] != 2 {
        return None;
    }
    let header_size = data[5] as usize;
    if header_size < HEADER_V2 || header_size > 128 || data.len() < header_size {
        return None;
    }
    let node_id = data[6];
    let n_antennas = data[7].max(1);
    let station_id = u16::from_le_bytes(data[8..10].try_into().ok()?);
    let link_id16 = u16::from_le_bytes(data[10..12].try_into().ok()?);
    let n_subcarriers = u16::from_le_bytes(data[12..14].try_into().ok()?);
    let channel = data[14];
    let flags = data[15] as u16;
    let rssi = data[16] as i8;
    let noise_floor = data[17] as i8;
    let firmware_version_code = u16::from_le_bytes(data[18..20].try_into().ok()?);
    let sequence = u32::from_le_bytes(data[20..24].try_into().ok()?);
    let agent_sequence = u32::from_le_bytes(data[24..28].try_into().ok()?);
    let timestamp_us = u64::from_le_bytes(data[28..36].try_into().ok()?);
    let mut source_mac_hash = [0u8; 8];
    source_mac_hash.copy_from_slice(&data[36..44]);
    let payload_length = u16::from_le_bytes(data[44..46].try_into().ok()?) as usize;
    let payload_crc16 = u16::from_le_bytes(data[46..48].try_into().ok()?);
    let expected = n_subcarriers as usize * 2 * n_antennas as usize;
    if station_id == 0
        || n_subcarriers == 0
        || expected > 8_192
        || payload_length < expected
        || data.len() < header_size + expected
    {
        return None;
    }
    let payload = &data[header_size..header_size + expected];
    if payload_crc16 != 0 && crc16_ccitt(payload) != payload_crc16 {
        return None;
    }
    let iq_data = payload.iter().map(|value| *value as i8).collect::<Vec<_>>();
    let (amplitudes, phases) = calculate_features(&iq_data, n_subcarriers as usize);
    Some(CsiFrame {
        protocol_version: 2,
        node_id,
        station_id,
        link_id: if link_id16 == 0 {
            ((node_id as u32) << 16) | station_id as u32
        } else {
            link_id16 as u32
        },
        n_antennas,
        n_subcarriers,
        frequency_mhz: frequency_from_channel(channel),
        channel,
        sequence,
        agent_sequence,
        rssi,
        noise_floor,
        flags,
        firmware_version_code,
        timestamp_us,
        received_at_ms: now_ms(),
        source_mac_hash,
        payload_crc16,
        iq_data,
        amplitudes,
        phases,
    })
}

pub fn stable_station_id_from_mac_hash(hash: &[u8; 8]) -> u16 {
    let mut value = 0x811cu16;
    for byte in hash {
        value ^= *byte as u16;
        value = value.wrapping_mul(0x0193);
    }
    value.max(1)
}

pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc = 0xffffu16;
    for byte in data {
        crc ^= (*byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

pub fn build_v2_test_frame(node_id: u8, station_id: u16, sequence: u32) -> Vec<u8> {
    let n_subcarriers = 56u16;
    let payload_length = n_subcarriers as usize * 2;
    let mut output = vec![0u8; HEADER_V2 + payload_length];
    output[0..4].copy_from_slice(&MAGIC_V2.to_le_bytes());
    output[4] = 2;
    output[5] = HEADER_V2 as u8;
    output[6] = node_id;
    output[7] = 1;
    output[8..10].copy_from_slice(&station_id.max(1).to_le_bytes());
    let link_id = (((node_id as u16) << 8) ^ station_id).max(1);
    output[10..12].copy_from_slice(&link_id.to_le_bytes());
    output[12..14].copy_from_slice(&n_subcarriers.to_le_bytes());
    output[14] = 6;
    output[15] = 1;
    output[16] = (-45i8) as u8;
    output[17] = (-92i8) as u8;
    output[18..20].copy_from_slice(&0x0201u16.to_le_bytes());
    output[20..24].copy_from_slice(&sequence.to_le_bytes());
    output[24..28].copy_from_slice(&sequence.to_le_bytes());
    output[28..36].copy_from_slice(&(sequence as u64 * 20_000).to_le_bytes());
    output[36..44].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
    output[44..46].copy_from_slice(&(payload_length as u16).to_le_bytes());
    for index in 0..payload_length / 2 {
        let phase = sequence as f32 * 0.12 + index as f32 * 0.2;
        output[HEADER_V2 + index * 2] = (phase.sin() * 70.0) as i8 as u8;
        output[HEADER_V2 + index * 2 + 1] = (phase.cos() * 70.0) as i8 as u8;
    }
    let crc = crc16_ccitt(&output[HEADER_V2..]);
    output[46..48].copy_from_slice(&crc.to_le_bytes());
    output
}

pub fn send_test_frames(
    target: &str,
    count: usize,
    node_id: u8,
    station_id: u16,
) -> anyhow::Result<()> {
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    for sequence in 0..count {
        let frame = build_v2_test_frame(node_id, station_id, sequence as u32);
        socket.send_to(&frame, target)?;
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn v2_roundtrip_validates_crc_and_provenance() {
        let bytes = build_v2_test_frame(2, 7, 11);
        let frame = parse_adr018(&bytes).expect("v2 frame");
        assert_eq!(frame.protocol_version, 2);
        assert_eq!(frame.node_id, 2);
        assert_eq!(frame.station_id, 7);
        assert_eq!(frame.sequence, 11);
        assert_eq!(frame.firmware_version_code, 0x0201);
        assert_eq!(frame.amplitudes.len(), 56);
        assert_ne!(frame.payload_crc16, 0);
    }

    #[test]
    fn corrupted_payload_is_rejected() {
        let mut bytes = build_v2_test_frame(2, 7, 11);
        *bytes.last_mut().unwrap() ^= 0xff;
        assert!(parse_adr018(&bytes).is_none());
    }

    #[test]
    fn bad_magic_is_rejected() {
        assert!(parse_adr018(&[1, 2, 3, 4, 5]).is_none());
    }

    #[test]
    fn stable_station_id_is_nonzero_and_repeatable() {
        let hash = [9, 8, 7, 6, 5, 4, 3, 2];
        assert_eq!(stable_station_id_from_mac_hash(&hash), stable_station_id_from_mac_hash(&hash));
        assert_ne!(stable_station_id_from_mac_hash(&hash), 0);
    }
}

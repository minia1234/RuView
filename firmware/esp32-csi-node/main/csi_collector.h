/**
 * @file csi_collector.h
 * @brief CSI collection and ADR-018 v1/v2 binary frame serialization.
 */

#ifndef CSI_COLLECTOR_H
#define CSI_COLLECTOR_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_wifi_types.h"

/* Both serializers are intentionally compiled so one firmware source can switch
 * between v1 and v2 through CONFIG_FORMMAP_ADR018_V2. ESP-IDF otherwise treats
 * the inactive static serializer as an error under strict warning settings. The
 * I/Q length is bounded by the frame buffer; GCC can also diagnose the explicit
 * uint16 upper-bound guard as a type-limit comparison. */
#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wtype-limits"
#endif

/** Legacy ADR-018 magic number. */
#define CSI_MAGIC_V1 0xC5110001u

/** FormMap station-aware ADR-018 v2 magic number. */
#define CSI_MAGIC_V2 0xC5110002u

/** Backward-compatible alias used by existing tests/modules. */
#define CSI_MAGIC CSI_MAGIC_V1

/** ADR-018 header sizes. */
#define CSI_HEADER_V1_SIZE 20u
#define CSI_HEADER_V2_SIZE 48u

#ifndef CONFIG_FORMMAP_ADR018_V2
#define CONFIG_FORMMAP_ADR018_V2 1
#endif

#ifndef CONFIG_FORMMAP_FIRMWARE_VERSION_CODE
#define CONFIG_FORMMAP_FIRMWARE_VERSION_CODE 0x0201
#endif

#ifndef CONFIG_FORMMAP_STATION_TABLE_SIZE
#define CONFIG_FORMMAP_STATION_TABLE_SIZE 16
#endif

#ifndef CONFIG_FORMMAP_MAC_HASH_SALT
#define CONFIG_FORMMAP_MAC_HASH_SALT "ruview-formmap-local-salt"
#endif

#if CONFIG_FORMMAP_ADR018_V2
#define CSI_HEADER_SIZE CSI_HEADER_V2_SIZE
#else
#define CSI_HEADER_SIZE CSI_HEADER_V1_SIZE
#endif

/** Maximum frame buffer size (v2 header + 4 antennas * 256 subcarriers * I/Q). */
#define CSI_MAX_FRAME_SIZE (CSI_HEADER_V2_SIZE + 4u * 256u * 2u)

/** Maximum number of channels in the hop table (ADR-029). */
#define CSI_HOP_CHANNELS_MAX 6

/** Initialize CSI collection and register the ESP-IDF callback. */
void csi_collector_init(void);

/** Capture node ID before Wi-Fi initialization can mutate NVS-backed state. */
void csi_collector_set_node_id(uint8_t node_id);

/** Return the authoritative runtime node ID. */
uint8_t csi_collector_get_node_id(void);

/**
 * Serialize one ESP-IDF CSI callback as ADR-018 v1 or v2.
 *
 * v2 includes deterministic station identity derived from info->mac, a salted
 * source-MAC hash, per-station sequence, local timestamp, firmware provenance
 * and CRC16 over the raw I/Q payload.
 */
size_t csi_serialize_frame(const wifi_csi_info_t *info, uint8_t *buf, size_t buf_len);

/** Compute the salted privacy-preserving 64-bit hash used on the wire. */
void csi_formmap_hash_mac(const uint8_t mac[6], uint8_t out_hash[8]);

/** Convert the salted hash to a stable non-zero station ID. */
uint16_t csi_formmap_station_id(const uint8_t hash[8]);

/** CRC16-CCITT used by ADR-018 v2 payload integrity. */
uint16_t csi_formmap_crc16(const uint8_t *data, size_t len);

/** Configure channel-hop table for ADR-029. */
void csi_collector_set_hop_table(const uint8_t *channels, uint8_t hop_count, uint32_t dwell_ms);

/** Advance to the next configured channel. */
void csi_hop_next_channel(void);

/** Start the periodic channel-hop timer. */
void csi_collector_start_hop_timer(void);

/** Enable DATA-frame capture in addition to management frames. */
void csi_collector_enable_data_capture(void);

/** Inject the existing null-data sensing probe placeholder. */
esp_err_t csi_inject_ndp_frame(void);

/** Recent CSI callback yield per second. */
uint16_t csi_collector_get_pkt_yield_per_sec(void);

/** Cumulative UDP send failures. */
uint16_t csi_collector_get_send_fail_count(void);

#endif /* CSI_COLLECTOR_H */

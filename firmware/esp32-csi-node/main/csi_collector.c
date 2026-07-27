/**
 * @file csi_collector.c
 * @brief CSI collection with FormMap station-aware ADR-018 v2 output.
 *
 * Preserves the existing RuView channel, rate-limit, self-ping, edge-processing,
 * cross-node sync and channel-hop behavior while adding deterministic source
 * station identity, privacy-preserving MAC hashing, per-station sequence numbers,
 * firmware provenance and CRC16 payload integrity.
 */

#include "csi_collector.h"
#include "nvs_config.h"
#include "stream_sender.h"
#include "edge_processing.h"
#include "c6_timesync.h"
#include "c6_sync_espnow.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "ping/ping_sock.h"
#include "sdkconfig.h"
#include "lwip/ip_addr.h"

extern nvs_config_t g_nvs_config;

#ifndef CONFIG_ESP_WIFI_CSI_ENABLED
#error "CONFIG_ESP_WIFI_CSI_ENABLED must be enabled for the CSI node firmware"
#endif

#define CSI_MIN_SEND_INTERVAL_US       (20 * 1000)
#define CSI_MIN_PROCESS_INTERVAL_US    (20 * 1000)
#define CSI_STATION_TABLE_SIZE         CONFIG_FORMMAP_STATION_TABLE_SIZE
#define CSI_STATION_IDENTITY_FLAG      (1u << 5)
#define CSI_SYNC_VALID_FLAG            (1u << 4)
#define CSI_BW40_FLAG                  (1u << 0)
#define CSI_STBC_FLAG                  (1u << 2)

static const char *TAG = "csi_collector";

static uint8_t s_node_id = 1;
static bool s_node_id_early_set = false;
static uint8_t s_filter_mac[6] = {0};
static bool s_filter_mac_set = false;

static uint32_t s_global_sequence = 0;
static uint32_t s_cb_count = 0;
static uint32_t s_send_ok = 0;
static uint32_t s_send_fail = 0;
static uint32_t s_rate_skip = 0;
static uint32_t s_early_drop = 0;
static int64_t s_last_send_us = 0;
static int64_t s_last_process_us = 0;

static uint8_t s_hop_channels[CSI_HOP_CHANNELS_MAX] = {1, 6, 11, 36, 40, 44};
static uint8_t s_hop_count = 1;
static uint32_t s_dwell_ms = 50;
static uint8_t s_hop_index = 0;
static esp_timer_handle_t s_hop_timer = NULL;
static esp_ping_handle_t s_self_ping = NULL;

typedef struct {
    bool used;
    uint8_t hash[8];
    uint16_t station_id;
    uint32_t next_sequence;
    uint32_t last_seen_tick;
} formmap_station_slot_t;

static formmap_station_slot_t s_station_table[CSI_STATION_TABLE_SIZE];
static uint32_t s_station_tick = 0;

static uint32_t channel_to_frequency(uint8_t channel)
{
    if (channel >= 1 && channel <= 13) {
        return 2412u + ((uint32_t)channel - 1u) * 5u;
    }
    if (channel == 14) {
        return 2484u;
    }
    if (channel >= 36 && channel <= 177) {
        return 5000u + (uint32_t)channel * 5u;
    }
    return 0;
}

uint16_t csi_formmap_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xffffu;
    if (data == NULL) {
        return crc;
    }
    for (size_t index = 0; index < len; ++index) {
        crc ^= (uint16_t)data[index] << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void csi_formmap_hash_mac(const uint8_t mac[6], uint8_t out_hash[8])
{
    /* Salted FNV-1a is used as a privacy pseudonym, not as authentication.
     * Deployments should override CONFIG_FORMMAP_MAC_HASH_SALT per installation. */
    uint64_t hash = 1469598103934665603ULL;
    const char *salt = CONFIG_FORMMAP_MAC_HASH_SALT;
    if (salt != NULL) {
        for (size_t index = 0; salt[index] != '\0'; ++index) {
            hash ^= (uint8_t)salt[index];
            hash *= 1099511628211ULL;
        }
    }
    if (mac != NULL) {
        for (size_t index = 0; index < 6; ++index) {
            hash ^= mac[index];
            hash *= 1099511628211ULL;
        }
    }
    for (size_t index = 0; index < 8; ++index) {
        out_hash[index] = (uint8_t)((hash >> (index * 8u)) & 0xffu);
    }
}

uint16_t csi_formmap_station_id(const uint8_t hash[8])
{
    uint16_t value = 0x811cu;
    for (size_t index = 0; index < 8; ++index) {
        value ^= hash[index];
        value = (uint16_t)(value * 0x0193u);
    }
    return value == 0 ? 1 : value;
}

static formmap_station_slot_t *station_slot_for_hash(const uint8_t hash[8])
{
    formmap_station_slot_t *empty = NULL;
    formmap_station_slot_t *oldest = &s_station_table[0];
    for (size_t index = 0; index < CSI_STATION_TABLE_SIZE; ++index) {
        formmap_station_slot_t *slot = &s_station_table[index];
        if (slot->used && memcmp(slot->hash, hash, 8) == 0) {
            slot->last_seen_tick = ++s_station_tick;
            return slot;
        }
        if (!slot->used && empty == NULL) {
            empty = slot;
        }
        if (slot->last_seen_tick < oldest->last_seen_tick) {
            oldest = slot;
        }
    }

    formmap_station_slot_t *slot = empty != NULL ? empty : oldest;
    memset(slot, 0, sizeof(*slot));
    slot->used = true;
    memcpy(slot->hash, hash, 8);
    slot->station_id = csi_formmap_station_id(hash);
    slot->next_sequence = 0;
    slot->last_seen_tick = ++s_station_tick;
    return slot;
}

static uint8_t formmap_frame_flags(const wifi_csi_info_t *info)
{
    uint8_t flags = CSI_STATION_IDENTITY_FLAG;
#if defined(CONFIG_SOC_WIFI_HE_SUPPORT) && CONFIG_SOC_WIFI_HE_SUPPORT
    if (info->rx_ctrl.second != 0) {
        flags |= CSI_BW40_FLAG;
    }
#else
    if (info->rx_ctrl.cwb) {
        flags |= CSI_BW40_FLAG;
    }
    if (info->rx_ctrl.stbc) {
        flags |= CSI_STBC_FLAG;
    }
#endif
#if defined(CONFIG_IDF_TARGET_ESP32C6) && defined(CONFIG_C6_TIMESYNC_ENABLE)
    if (c6_timesync_is_valid()) {
        flags |= CSI_SYNC_VALID_FLAG;
    }
#endif
    if (c6_sync_espnow_is_valid()) {
        flags |= CSI_SYNC_VALID_FLAG;
    }
    return flags;
}

static size_t serialize_v1(const wifi_csi_info_t *info, uint8_t *buf, size_t buf_len)
{
    const uint8_t antennas = 1;
    const uint16_t iq_len = (uint16_t)info->len;
    const uint16_t subcarriers = iq_len / (2u * antennas);
    const size_t frame_size = CSI_HEADER_V1_SIZE + iq_len;
    if (subcarriers == 0 || frame_size > buf_len) {
        return 0;
    }

    const uint32_t magic = CSI_MAGIC_V1;
    const uint32_t frequency_mhz = channel_to_frequency(info->rx_ctrl.channel);
    const uint32_t sequence = s_global_sequence++;
    memcpy(&buf[0], &magic, 4);
    buf[4] = s_node_id;
    buf[5] = antennas;
    memcpy(&buf[6], &subcarriers, 2);
    memcpy(&buf[8], &frequency_mhz, 4);
    memcpy(&buf[12], &sequence, 4);
    buf[16] = (uint8_t)(int8_t)info->rx_ctrl.rssi;
    buf[17] = (uint8_t)(int8_t)info->rx_ctrl.noise_floor;
    buf[18] = 0;
    buf[19] = formmap_frame_flags(info) & 0x1fu;
    memcpy(&buf[CSI_HEADER_V1_SIZE], info->buf, iq_len);
    return frame_size;
}

static size_t serialize_v2(const wifi_csi_info_t *info, uint8_t *buf, size_t buf_len)
{
    const uint8_t antennas = 1;
    const uint16_t iq_len = (uint16_t)info->len;
    const uint16_t subcarriers = iq_len / (2u * antennas);
    const size_t frame_size = CSI_HEADER_V2_SIZE + iq_len;
    if (subcarriers == 0 || frame_size > buf_len || iq_len > UINT16_MAX) {
        return 0;
    }

    uint8_t source_hash[8];
    csi_formmap_hash_mac(info->mac, source_hash);
    formmap_station_slot_t *slot = station_slot_for_hash(source_hash);
    const uint16_t station_id = slot->station_id;
    uint16_t link_id = (uint16_t)(((uint16_t)s_node_id << 8) ^ station_id);
    if (link_id == 0) {
        link_id = 1;
    }
    const uint32_t magic = CSI_MAGIC_V2;
    const uint32_t global_sequence = s_global_sequence++;
    const uint32_t station_sequence = slot->next_sequence++;
    const uint64_t timestamp_us = (uint64_t)esp_timer_get_time();
    const uint16_t firmware_version = (uint16_t)CONFIG_FORMMAP_FIRMWARE_VERSION_CODE;
    const uint16_t payload_len = iq_len;
    const uint16_t payload_crc = csi_formmap_crc16((const uint8_t *)info->buf, iq_len);

    memset(buf, 0, CSI_HEADER_V2_SIZE);
    memcpy(&buf[0], &magic, 4);
    buf[4] = 2;
    buf[5] = CSI_HEADER_V2_SIZE;
    buf[6] = s_node_id;
    buf[7] = antennas;
    memcpy(&buf[8], &station_id, 2);
    memcpy(&buf[10], &link_id, 2);
    memcpy(&buf[12], &subcarriers, 2);
    buf[14] = info->rx_ctrl.channel;
    buf[15] = formmap_frame_flags(info);
    buf[16] = (uint8_t)(int8_t)info->rx_ctrl.rssi;
    buf[17] = (uint8_t)(int8_t)info->rx_ctrl.noise_floor;
    memcpy(&buf[18], &firmware_version, 2);
    memcpy(&buf[20], &global_sequence, 4);
    memcpy(&buf[24], &station_sequence, 4);
    memcpy(&buf[28], &timestamp_us, 8);
    memcpy(&buf[36], source_hash, 8);
    memcpy(&buf[44], &payload_len, 2);
    memcpy(&buf[46], &payload_crc, 2);
    memcpy(&buf[CSI_HEADER_V2_SIZE], info->buf, iq_len);
    return frame_size;
}

size_t csi_serialize_frame(const wifi_csi_info_t *info, uint8_t *buf, size_t buf_len)
{
    if (info == NULL || buf == NULL || info->buf == NULL || info->len <= 0) {
        return 0;
    }
#if CONFIG_FORMMAP_ADR018_V2
    return serialize_v2(info, buf, buf_len);
#else
    return serialize_v1(info, buf, buf_len);
#endif
}

static void emit_sync_packet(void)
{
#ifndef CONFIG_C6_SYNC_EVERY_N_FRAMES
#define CONFIG_C6_SYNC_EVERY_N_FRAMES 20
#endif
    if ((s_cb_count % CONFIG_C6_SYNC_EVERY_N_FRAMES) != 0) {
        return;
    }
    uint8_t packet[32] = {0};
    const uint32_t magic = 0xC511A110u;
    const uint64_t local_us = (uint64_t)esp_timer_get_time();
    const uint64_t epoch_us = c6_sync_espnow_get_epoch_us();
    const int64_t offset = c6_sync_espnow_get_offset_us_smoothed();
    uint8_t flags = 0;
    if (c6_sync_espnow_is_leader()) {
        flags |= 0x01;
    }
    if (c6_sync_espnow_is_valid()) {
        flags |= 0x02;
    }
    if (offset != 0) {
        flags |= 0x04;
    }
    memcpy(&packet[0], &magic, 4);
    packet[4] = s_node_id;
    packet[5] = 1;
    packet[6] = flags;
    memcpy(&packet[8], &local_us, 8);
    memcpy(&packet[16], &epoch_us, 8);
    memcpy(&packet[24], &s_global_sequence, 4);
    (void)stream_sender_send_priority(packet, sizeof(packet));
}

static void wifi_csi_callback(void *ctx, wifi_csi_info_t *info)
{
    (void)ctx;
    if (info == NULL || info->buf == NULL || info->len <= 0) {
        return;
    }

    const int64_t now_us = esp_timer_get_time();
    if ((now_us - s_last_process_us) < CSI_MIN_PROCESS_INTERVAL_US) {
        ++s_early_drop;
        return;
    }
    s_last_process_us = now_us;

    if (s_filter_mac_set && memcmp(info->mac, s_filter_mac, 6) != 0) {
        return;
    }

    ++s_cb_count;
    uint8_t frame_buf[CSI_MAX_FRAME_SIZE];
    const size_t frame_len = csi_serialize_frame(info, frame_buf, sizeof(frame_buf));
    if (frame_len > 0) {
        const int64_t send_now = esp_timer_get_time();
        if ((send_now - s_last_send_us) >= CSI_MIN_SEND_INTERVAL_US) {
            const int result = stream_sender_send(frame_buf, frame_len);
            if (result > 0) {
                ++s_send_ok;
                s_last_send_us = send_now;
            } else {
                ++s_send_fail;
                if (s_send_fail <= 5 || (s_send_fail % 100) == 0) {
                    ESP_LOGW(TAG, "CSI send failed (#%lu)", (unsigned long)s_send_fail);
                }
            }
        } else {
            ++s_rate_skip;
        }
    }

    edge_enqueue_csi((const uint8_t *)info->buf,
                     (uint16_t)info->len,
                     (int8_t)info->rx_ctrl.rssi,
                     info->rx_ctrl.channel);
    emit_sync_packet();

    if (s_cb_count <= 3 || (s_cb_count % 250) == 0) {
        uint8_t hash[8];
        csi_formmap_hash_mac(info->mac, hash);
        ESP_LOGI(TAG,
                 "CSI #%lu v%d node=%u station=%u len=%d rssi=%d ch=%d sent=%lu drop=%lu",
                 (unsigned long)s_cb_count,
                 CONFIG_FORMMAP_ADR018_V2 ? 2 : 1,
                 (unsigned)s_node_id,
                 (unsigned)csi_formmap_station_id(hash),
                 info->len,
                 info->rx_ctrl.rssi,
                 info->rx_ctrl.channel,
                 (unsigned long)s_send_ok,
                 (unsigned long)s_early_drop);
    }
}

static void wifi_promiscuous_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    (void)buf;
    (void)type;
}

static void csi_ping_cb_noop(esp_ping_handle_t handle, void *args)
{
    (void)handle;
    (void)args;
}

static void csi_start_self_ping(void)
{
    if (s_self_ping != NULL) {
        return;
    }
    esp_netif_t *station = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip;
    if (station == NULL || esp_netif_get_ip_info(station, &ip) != ESP_OK || ip.gw.addr == 0) {
        ESP_LOGW(TAG, "self-ping unavailable; CSI relies on ambient OFDM frames");
        return;
    }

    char gateway[16];
    esp_ip4addr_ntoa(&ip.gw, gateway, sizeof(gateway));
    ip_addr_t target;
    memset(&target, 0, sizeof(target));
    ipaddr_aton(gateway, &target);

    esp_ping_config_t config = ESP_PING_DEFAULT_CONFIG();
    config.target_addr = target;
    config.count = ESP_PING_COUNT_INFINITE;
    config.interval_ms = 20;
    config.data_size = 1;
    config.task_stack_size = 4096;
    const esp_ping_callbacks_t callbacks = {
        .cb_args = NULL,
        .on_ping_success = csi_ping_cb_noop,
        .on_ping_timeout = csi_ping_cb_noop,
        .on_ping_end = csi_ping_cb_noop,
    };
    if (esp_ping_new_session(&config, &callbacks, &s_self_ping) == ESP_OK &&
        s_self_ping != NULL) {
        esp_ping_start(s_self_ping);
        ESP_LOGI(TAG, "self-ping started -> %s @50 Hz", gateway);
    } else {
        s_self_ping = NULL;
        ESP_LOGW(TAG, "failed to start self-ping");
    }
}

void csi_collector_set_node_id(uint8_t node_id)
{
    s_node_id = node_id;
    s_node_id_early_set = true;
    s_filter_mac_set = (g_nvs_config.filter_mac_set != 0);
    if (s_filter_mac_set) {
        memcpy(s_filter_mac, g_nvs_config.filter_mac, 6);
    }
    memset(s_station_table, 0, sizeof(s_station_table));
    ESP_LOGI(TAG,
             "node_id=%u captured; ADR-018 v%d firmware_code=0x%04x",
             (unsigned)s_node_id,
             CONFIG_FORMMAP_ADR018_V2 ? 2 : 1,
             (unsigned)CONFIG_FORMMAP_FIRMWARE_VERSION_CODE);
}

uint8_t csi_collector_get_node_id(void)
{
    return s_node_id;
}

void csi_collector_init(void)
{
    if (!s_node_id_early_set) {
        s_node_id = g_nvs_config.node_id;
        s_filter_mac_set = (g_nvs_config.filter_mac_set != 0);
        if (s_filter_mac_set) {
            memcpy(s_filter_mac, g_nvs_config.filter_mac, 6);
        }
        ESP_LOGW(TAG, "late node/config capture; call set_node_id before Wi-Fi init");
    }

    uint8_t csi_channel = (uint8_t)CONFIG_CSI_WIFI_CHANNEL;
    if (g_nvs_config.csi_channel > 0) {
        csi_channel = g_nvs_config.csi_channel;
    } else {
        wifi_ap_record_t access_point;
        if (esp_wifi_sta_get_ap_info(&access_point) == ESP_OK && access_point.primary > 0) {
            csi_channel = access_point.primary;
        }
    }
    s_hop_channels[0] = csi_channel;

    const esp_err_t power_result = esp_wifi_set_ps(WIFI_PS_NONE);
    if (power_result != ESP_OK) {
        ESP_LOGW(TAG, "WIFI_PS_NONE failed: %s", esp_err_to_name(power_result));
    }

    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_cb));
    const wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT,
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));

#if defined(CONFIG_SOC_WIFI_HE_SUPPORT) && CONFIG_SOC_WIFI_HE_SUPPORT
    wifi_csi_config_t csi_config;
    memset(&csi_config, 0, sizeof(csi_config));
    csi_config.enable = 1U;
    csi_config.acquire_csi_legacy = 1U;
    csi_config.acquire_csi_ht20 = 1U;
    csi_config.acquire_csi_ht40 = 1U;
    csi_config.acquire_csi_su = 1U;
    csi_config.acquire_csi_mu = 1U;
    csi_config.acquire_csi_dcm = 1U;
    csi_config.acquire_csi_beamformed = 1U;
#if defined(CONFIG_SOC_WIFI_MAC_VERSION_NUM) && CONFIG_SOC_WIFI_MAC_VERSION_NUM >= 3
    csi_config.acquire_csi_force_lltf = 1U;
    csi_config.acquire_csi_vht = 1U;
    csi_config.acquire_csi_he_stbc_mode = ESP_CSI_ACQUIRE_STBC_SAMPLE_HELTFS;
    csi_config.val_scale_cfg = 0U;
#else
    csi_config.acquire_csi_he_stbc = ESP_CSI_ACQUIRE_STBC_SAMPLE_HELTFS;
    csi_config.val_scale_cfg = 0U;
#endif
    csi_config.dump_ack_en = 0U;
#else
    const wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = false,
        .manu_scale = false,
        .shift = false,
    };
#endif

    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_callback, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
    csi_start_self_ping();

    ESP_LOGI(TAG,
             "CSI initialized node=%u channel=%u ADR-018=v%d station_slots=%u",
             (unsigned)s_node_id,
             (unsigned)csi_channel,
             CONFIG_FORMMAP_ADR018_V2 ? 2 : 1,
             (unsigned)CSI_STATION_TABLE_SIZE);
}

uint16_t csi_collector_get_pkt_yield_per_sec(void)
{
    static int64_t window_start_us = 0;
    static uint32_t window_start_count = 0;
    static uint16_t last_yield = 0;
    const int64_t now = esp_timer_get_time();
    if (window_start_us == 0) {
        window_start_us = now;
        window_start_count = s_cb_count;
        return 0;
    }
    const int64_t elapsed = now - window_start_us;
    if (elapsed < 1000000LL) {
        return last_yield;
    }
    const uint32_t delta = s_cb_count - window_start_count;
    uint64_t per_second = ((uint64_t)delta * 1000000ULL) / (uint64_t)elapsed;
    if (per_second > UINT16_MAX) {
        per_second = UINT16_MAX;
    }
    last_yield = (uint16_t)per_second;
    window_start_us = now;
    window_start_count = s_cb_count;
    return last_yield;
}

uint16_t csi_collector_get_send_fail_count(void)
{
    return s_send_fail > UINT16_MAX ? UINT16_MAX : (uint16_t)s_send_fail;
}

void csi_collector_set_hop_table(const uint8_t *channels,
                                 uint8_t hop_count,
                                 uint32_t dwell_ms)
{
    if (channels == NULL || hop_count == 0 || hop_count > CSI_HOP_CHANNELS_MAX) {
        ESP_LOGW(TAG, "invalid channel-hop table");
        return;
    }
    if (dwell_ms < 10) {
        dwell_ms = 10;
    }
    memcpy(s_hop_channels, channels, hop_count);
    s_hop_count = hop_count;
    s_dwell_ms = dwell_ms;
    s_hop_index = 0;
}

void csi_hop_next_channel(void)
{
    if (s_hop_count <= 1) {
        return;
    }
    s_hop_index = (uint8_t)((s_hop_index + 1u) % s_hop_count);
    const uint8_t channel = s_hop_channels[s_hop_index];
    const esp_err_t result = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "channel hop to %u failed: %s",
                 (unsigned)channel,
                 esp_err_to_name(result));
    }
}

static void hop_timer_callback(void *arg)
{
    (void)arg;
    csi_hop_next_channel();
}

void csi_collector_start_hop_timer(void)
{
    if (s_hop_count <= 1 || s_hop_timer != NULL) {
        return;
    }
    const esp_timer_create_args_t timer_args = {
        .callback = hop_timer_callback,
        .arg = NULL,
        .name = "csi_hop",
    };
    esp_err_t result = esp_timer_create(&timer_args, &s_hop_timer);
    if (result != ESP_OK) {
        s_hop_timer = NULL;
        ESP_LOGE(TAG, "failed to create hop timer: %s", esp_err_to_name(result));
        return;
    }
    result = esp_timer_start_periodic(s_hop_timer, (uint64_t)s_dwell_ms * 1000ULL);
    if (result != ESP_OK) {
        esp_timer_delete(s_hop_timer);
        s_hop_timer = NULL;
        ESP_LOGE(TAG, "failed to start hop timer: %s", esp_err_to_name(result));
    }
}

void csi_collector_enable_data_capture(void)
{
    const wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA,
    };
    const esp_err_t result = esp_wifi_set_promiscuous_filter(&filter);
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "CSI filter upgraded to MGMT+DATA");
    } else {
        ESP_LOGW(TAG, "failed to enable DATA capture: %s", esp_err_to_name(result));
    }
}

esp_err_t csi_inject_ndp_frame(void)
{
    uint8_t frame[24] = {0};
    frame[0] = 0x48;
    frame[1] = 0x00;
    memset(&frame[4], 0xff, 6);
    memset(&frame[16], 0xff, 6);
    const esp_err_t result = esp_wifi_80211_tx(WIFI_IF_STA, frame, sizeof(frame), false);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "NDP/null-data inject failed: %s", esp_err_to_name(result));
    }
    return result;
}

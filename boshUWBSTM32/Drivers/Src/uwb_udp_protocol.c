#include "uwb_udp_protocol.h"

#include <string.h>
#include <stdio.h>

#include "main.h"
#include "udp_server.h"
#include "uci/uci_radar.h"
#include "uci/uci_types.h"

typedef struct {
    uci_radar_params_t current_cfg;
    uint32_t session_handle;
    bool session_initialized;
    bool session_config_valid;
    bool session_running;
    uint32_t session_duration_ms;  /* 0 = unlimited */
    uint32_t session_start_tick;
    uint32_t session_last_debug_tick;
    uint32_t session_last_frame_rx_tick;
    uint32_t session_last_frame_tx_tick;
    uint32_t session_last_frame_drop_tick;
    uint32_t debug_prev_frame_rx_count;
    uint32_t debug_prev_frame_tx_ok_count;
    uint32_t debug_prev_frame_tx_drop_count;
    uint32_t frame_rx_count;
    uint32_t frame_tx_ok_count;
    uint32_t frame_tx_drop_count;
    uint32_t queue_drop_count;
} uwb_protocol_state_t;

static uwb_protocol_state_t s_state;

#define UWB_TX_QUEUE_DEPTH 12U
#define UWB_TX_DRAIN_BUDGET 4U

typedef struct {
    uint16_t raw_len;
    uint8_t raw_payload[UWB_RADAR_RAW_MAX_LEN];
} uwb_radar_tx_item_t;

static uwb_radar_tx_item_t s_tx_queue[UWB_TX_QUEUE_DEPTH];
static uint8_t s_tx_queue_head;
static uint8_t s_tx_queue_tail;
static uint8_t s_tx_queue_count;
static uint8_t s_tx_queue_high_water;

static uint16_t get_le16(const uint8_t *buf)
{
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static uint32_t get_le32(const uint8_t *buf)
{
    return (uint32_t)buf[0]
         | ((uint32_t)buf[1] << 8)
         | ((uint32_t)buf[2] << 16)
         | ((uint32_t)buf[3] << 24);
}

static void put_le16(uint8_t *buf, uint16_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
}

static void put_le32(uint8_t *buf, uint32_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

static uint16_t uwb_build_envelope(uint8_t *buf, uint8_t msg_type,
                                    uint16_t seq, const uint8_t *payload, uint16_t payload_len)
{
    buf[0] = UWB_UDP_MAGIC_0;
    buf[1] = UWB_UDP_MAGIC_1;
    buf[2] = UWB_UDP_PROTOCOL_VERSION;
    buf[3] = msg_type;
    put_le16(&buf[4], seq);
    put_le16(&buf[6], payload_len);
    if (payload_len > 0 && payload != NULL) {
        memcpy(&buf[8], payload, payload_len);
    }
    return (uint16_t)(8 + payload_len);
}

static void uwb_send_ack(uint16_t reply_port, uint16_t seq, uint8_t acked_msg_type)
{
    uint8_t payload[3];
    uint8_t packet[16];

    (void)reply_port;  /* always reply to the fixed PC listener port */

    payload[0] = acked_msg_type;
    put_le16(&payload[1], seq);

    uint16_t packet_len = uwb_build_envelope(packet, UWB_MSG_ACK, seq, payload, sizeof(payload));
    printf("[UWB] TX ACK msg=0x%02X seq=%u port=%u len=%u\n\r",
           acked_msg_type, seq, (unsigned)UWB_ACK_REPLY_PORT, packet_len);
    nucleo_udp_send(UWB_ACK_REPLY_PORT, packet, packet_len);
}

static void uwb_send_error(uint16_t reply_port, uint16_t seq,
                            uint8_t failed_msg_type, uint16_t error_code, uint8_t detail)
{
    uint8_t payload[6];
    uint8_t packet[20];

    (void)reply_port;  /* always reply to the fixed PC listener port */

    payload[0] = failed_msg_type;
    put_le16(&payload[1], seq);
    put_le16(&payload[3], error_code);
    payload[5] = detail;

    uint16_t packet_len = uwb_build_envelope(packet, UWB_MSG_ERROR, seq, payload, sizeof(payload));
    printf("[UWB] TX ERR failed_msg=0x%02X err=0x%04X detail=0x%02X port=%u\n\r",
           failed_msg_type, error_code, detail, (unsigned)UWB_ACK_REPLY_PORT);
    nucleo_udp_send(UWB_ACK_REPLY_PORT, packet, packet_len);
}

static uint32_t uwb_required_mask(void)
{
    return (1UL << 0)  /* CHANNEL_NUMBER */
         | (1UL << 1)  /* PREAMBLE_CODE_INDEX */
         | (1UL << 2)  /* ANTENNAS_CONFIG_RX */
         | (1UL << 3)  /* ANTENNAS_CONFIG_TX */
         | (1UL << 4)  /* RADAR_MODE */
         | (1UL << 5)  /* RADAR_SINGLE_FRAME_NTF */
         | (1UL << 6)  /* RADAR_RFRI */
         | (1UL << 7)  /* RADAR_CIR_NUM_SAMPLES */
         | (1UL << 8)  /* RADAR_RX_GAIN */
         | (1UL << 9)  /* RADAR_CIR_START_OFFSET */
         | (1UL << 10) /* RADAR_PERFORMANCE */
         | (1UL << 11) /* RADAR_DRIFT_COMP */
         | (1UL << 12);/* RADAR_PRESENCE_CFG */
}

static uci_status_t uwb_ensure_session_initialized(void)
{
    if (s_state.session_initialized) {
        return UCI_STATUS_OK;
    }

    uci_status_t st = uci_radar_session_init(UWB_SESSION_ID, &s_state.session_handle);
    if (st == UCI_STATUS_OK) {
        s_state.session_initialized = true;
    }
    return st;
}

static uci_status_t uwb_apply_config(bool restart_after_apply)
{
    uci_status_t st;
    bool was_running = s_state.session_running;
    bool should_restart = restart_after_apply && was_running;

    if (was_running) {
        st = uci_radar_stop(s_state.session_handle);
        if (st != UCI_STATUS_OK) {
            return st;
        }
        s_state.session_running = false;
    }

    st = uwb_ensure_session_initialized();
    if (st != UCI_STATUS_OK) {
        return st;
    }

    st = uci_radar_configure(s_state.session_handle, &s_state.current_cfg);
    if (st != UCI_STATUS_OK) {
        return st;
    }

    s_state.session_config_valid = true;

    if (should_restart) {
        st = uci_radar_start(s_state.session_handle);
        if (st != UCI_STATUS_OK) {
            return st;
        }
        s_state.session_running = true;
    }

    return UCI_STATUS_OK;
}

static int uwb_tag_to_mask_bit(uint16_t tag)
{
    switch (tag) {
    case UWB_TAG_CHANNEL_NUMBER: return 0;
    case UWB_TAG_PREAMBLE_CODE_INDEX: return 1;
    case UWB_TAG_ANTENNAS_CONFIG_RX: return 2;
    case UWB_TAG_ANTENNAS_CONFIG_TX: return 3;
    case UWB_TAG_RADAR_MODE: return 4;
    case UWB_TAG_RADAR_SINGLE_FRAME_NTF: return 5;
    case UWB_TAG_RADAR_RFRI: return 6;
    case UWB_TAG_RADAR_CIR_NUM_SAMPLES: return 7;
    case UWB_TAG_RADAR_RX_GAIN: return 8;
    case UWB_TAG_RADAR_CIR_START_OFFSET: return 9;
    case UWB_TAG_RADAR_PERFORMANCE: return 10;
    case UWB_TAG_RADAR_DRIFT_COMP: return 11;
    case UWB_TAG_RADAR_PRESENCE_CFG: return 12;
    default: return -1;
    }
}

static uci_status_t uwb_parse_tlvs_into_cfg(const uint8_t *payload, uint16_t payload_len,
                                             uci_radar_params_t *cfg, uint32_t *seen_mask,
                                             uint16_t *bad_tag)
{
    uint16_t offset = 0;

    while (offset < payload_len) {
        uint16_t tag;
        uint16_t len;
        int bit;

        if (offset + 4 > payload_len) {
            return UCI_STATUS_INVALID_MSG_SIZE;
        }

        tag = get_le16(&payload[offset]);
        len = get_le16(&payload[offset + 2]);
        offset += 4;

        if (offset + len > payload_len) {
            return UCI_STATUS_INVALID_MSG_SIZE;
        }

        bit = uwb_tag_to_mask_bit(tag);
        if (bit >= 0) {
            *seen_mask |= (1UL << bit);
        }

        switch (tag) {
        case UWB_TAG_CHANNEL_NUMBER:
            if (len != 1) return UCI_STATUS_INVALID_MSG_SIZE;
            cfg->channel_number = payload[offset];
            break;

        case UWB_TAG_PREAMBLE_CODE_INDEX:
            if (len != 1) return UCI_STATUS_INVALID_MSG_SIZE;
            cfg->preamble_code_index = payload[offset];
            break;

        case UWB_TAG_ANTENNAS_CONFIG_RX:
            if (len != 5) return UCI_STATUS_INVALID_MSG_SIZE;
            cfg->ant_rx_rxc_id = payload[offset + 2];
            cfg->ant_rx_rxb_id = payload[offset + 3];
            cfg->ant_rx_rxa_id = payload[offset + 4];
            break;

        case UWB_TAG_ANTENNAS_CONFIG_TX:
            if (len != 2) return UCI_STATUS_INVALID_MSG_SIZE;
            cfg->ant_tx_id = payload[offset + 1];
            break;

        case UWB_TAG_RADAR_MODE:
            if (len != 1) return UCI_STATUS_INVALID_MSG_SIZE;
            cfg->mode = (uci_radar_mode_t)payload[offset];
            break;

        case UWB_TAG_RADAR_SINGLE_FRAME_NTF:
            if (len != 1) return UCI_STATUS_INVALID_MSG_SIZE;
            cfg->single_frame_ntf = payload[offset];
            break;

        case UWB_TAG_RADAR_RFRI:
            if (len != 7) return UCI_STATUS_INVALID_MSG_SIZE;
            cfg->use_rfri = true;
            cfg->rfri.ranging_interval_ms = get_le32(&payload[offset]);
            cfg->rfri.slot_duration_rstu = get_le16(&payload[offset + 4]);
            cfg->rfri.slots_per_rr = payload[offset + 6];
            break;

        case UWB_TAG_RADAR_CIR_NUM_SAMPLES:
            if (len != 1) return UCI_STATUS_INVALID_MSG_SIZE;
            cfg->cir_num_samples = payload[offset];
            break;

        case UWB_TAG_RADAR_RX_GAIN:
            if (len != 4) return UCI_STATUS_INVALID_MSG_SIZE;
            cfg->rx_gain.agc_mode = payload[offset];
            cfg->rx_gain.gain_rxa = payload[offset + 1];
            cfg->rx_gain.gain_rxb = payload[offset + 2];
            cfg->rx_gain.gain_rxc = payload[offset + 3];
            break;

        case UWB_TAG_RADAR_CIR_START_OFFSET:
            if (len != 6) return UCI_STATUS_INVALID_MSG_SIZE;
            cfg->cir_start_offset.rxc_start_index = get_le16(&payload[offset]);
            cfg->cir_start_offset.rxb_start_index = get_le16(&payload[offset + 2]);
            cfg->cir_start_offset.rxa_start_index = get_le16(&payload[offset + 4]);
            break;

        case UWB_TAG_RADAR_PERFORMANCE:
            if (len != 1) return UCI_STATUS_INVALID_MSG_SIZE;
            cfg->performance = payload[offset];
            break;

        case UWB_TAG_RADAR_DRIFT_COMP:
            if (len != 2) return UCI_STATUS_INVALID_MSG_SIZE;
            cfg->drift_compensation = get_le16(&payload[offset]);
            break;

        case UWB_TAG_RADAR_PRESENCE_CFG:
            if (len != 12) return UCI_STATUS_INVALID_MSG_SIZE;
            cfg->presence_det.mode = payload[offset];
            cfg->presence_det.periodic_report = payload[offset + 1];
            cfg->presence_det.sensitivity_q4_4 = payload[offset + 2];
            cfg->presence_det.gpio_notify = payload[offset + 3];
            cfg->presence_det.distance_min_cm = get_le16(&payload[offset + 4]);
            cfg->presence_det.distance_max_cm = get_le16(&payload[offset + 6]);
            cfg->presence_det.hold_delay_ms = get_le16(&payload[offset + 8]);
            cfg->presence_det.angle_min_deg = (int8_t)payload[offset + 10];
            cfg->presence_det.angle_max_deg = (int8_t)payload[offset + 11];
            break;

        default:
            *bad_tag = tag;
            return UCI_STATUS_INVALID_PARAM;
        }

        offset += len;
    }

    return UCI_STATUS_OK;
}

void uwb_udp_protocol_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_tx_queue_head = 0U;
    s_tx_queue_tail = 0U;
    s_tx_queue_count = 0U;
    s_tx_queue_high_water = 0U;
}

void uwb_udp_protocol_handle_packet(const uint8_t *data, uint16_t len,
                                     const ip_addr_t *addr, uint16_t port)
{
    uint8_t msg_type;
    uint16_t seq;
    uint16_t payload_len;
    const uint8_t *payload;
    uint32_t seen_mask = 0;
    uint16_t bad_tag = 0;
    uci_status_t st = UCI_STATUS_OK;

    (void)addr;

    printf("[UWB] RX %u bytes from port %u\n\r", len, port);

    if (data == NULL || len < 8) {
        printf("[UWB] RX too short (%u)\n\r", len);
        return;
    }

    if (data[0] != UWB_UDP_MAGIC_0 || data[1] != UWB_UDP_MAGIC_1) {
        printf("[UWB] Bad magic: 0x%02X 0x%02X\n\r", data[0], data[1]);
        uwb_send_error(port, 0, 0, UWB_ERR_INVALID_MAGIC, 0);
        return;
    }

    if (data[2] != UWB_UDP_PROTOCOL_VERSION) {
        printf("[UWB] Bad version: 0x%02X\n\r", data[2]);
        uwb_send_error(port, get_le16(&data[4]), data[3], UWB_ERR_INVALID_VERSION, data[2]);
        return;
    }

    msg_type = data[3];
    seq = get_le16(&data[4]);
    payload_len = get_le16(&data[6]);

    printf("[UWB] msg=0x%02X seq=%u payload=%u\n\r", msg_type, seq, payload_len);

    if (len != (uint16_t)(8 + payload_len)) {
        printf("[UWB] Length mismatch: got %u expected %u\n\r", len, 8 + payload_len);
        uwb_send_error(port, seq, msg_type, UWB_ERR_INVALID_LENGTH, 0);
        return;
    }

    payload = &data[8];

    switch (msg_type) {
    case UWB_MSG_SET_CONFIG_FULL: {
        uci_radar_params_t new_cfg;
        memset(&new_cfg, 0, sizeof(new_cfg));
        new_cfg.max_measurements = 0;
        new_cfg.use_perform_lprf_cal = false;

        printf("[UWB] SET_CONFIG_FULL: parsing %u bytes of TLVs\n\r", payload_len);
        st = uwb_parse_tlvs_into_cfg(payload, payload_len, &new_cfg, &seen_mask, &bad_tag);
        if (st != UCI_STATUS_OK) {
            printf("[UWB] TLV parse error: st=0x%02X bad_tag=0x%04X\n\r", st, bad_tag);
            uwb_send_error(port, seq, msg_type,
                           (st == UCI_STATUS_INVALID_PARAM) ? UWB_ERR_UNSUPPORTED_TAG : UWB_ERR_INVALID_TLV,
                           (uint8_t)(bad_tag & 0xFF));
            return;
        }

        printf("[UWB] TLV parse OK: seen=0x%08lX req=0x%08lX\n\r",
               (unsigned long)seen_mask, (unsigned long)uwb_required_mask());
        if (seen_mask != uwb_required_mask()) {
            printf("[UWB] Missing required TLVs\n\r");
            uwb_send_error(port, seq, msg_type, UWB_ERR_MISSING_REQUIRED_FIELD, 0);
            return;
        }

        s_state.current_cfg = new_cfg;
        printf("[UWB] Applying config (session_init + configure)...\n\r");
        st = uwb_apply_config(false);
        if (st != UCI_STATUS_OK) {
            printf("[UWB] Apply config failed: 0x%02X\n\r", st);
            uwb_send_error(port, seq, msg_type, UWB_ERR_UCI_COMMAND_FAILED, st);
            return;
        }

        printf("[UWB] Config applied OK\n\r");
        uwb_send_ack(port, seq, msg_type);
        break;
    }

    case UWB_MSG_SET_PARAMS_PARTIAL: {
        if (!s_state.session_config_valid) {
            uwb_send_error(port, seq, msg_type, UWB_ERR_INVALID_STATE, 0);
            return;
        }

        uci_radar_params_t updated_cfg = s_state.current_cfg;
        st = uwb_parse_tlvs_into_cfg(payload, payload_len, &updated_cfg, &seen_mask, &bad_tag);
        if (st != UCI_STATUS_OK) {
            uwb_send_error(port, seq, msg_type,
                           (st == UCI_STATUS_INVALID_PARAM) ? UWB_ERR_UNSUPPORTED_TAG : UWB_ERR_INVALID_TLV,
                           (uint8_t)(bad_tag & 0xFF));
            return;
        }

        s_state.current_cfg = updated_cfg;
        st = uwb_apply_config(true);
        if (st != UCI_STATUS_OK) {
            uwb_send_error(port, seq, msg_type, UWB_ERR_UCI_COMMAND_FAILED, st);
            return;
        }

        uwb_send_ack(port, seq, msg_type);
        break;
    }

    case UWB_MSG_START_RADAR:
        printf("[UWB] START_RADAR: cfg_valid=%d sess_init=%d running=%d payload=%u\n\r",
               s_state.session_config_valid, s_state.session_initialized,
               s_state.session_running, payload_len);
        if (!s_state.session_config_valid || !s_state.session_initialized) {
            printf("[UWB] START_RADAR rejected: invalid state\n\r");
            uwb_send_error(port, seq, msg_type, UWB_ERR_INVALID_STATE, 0);
            return;
        }

        if (s_state.session_running) {
            printf("[UWB] START_RADAR: already running, sending ACK\n\r");
            uwb_send_ack(port, seq, msg_type);
            return;
        }

        if (payload_len == 4) {
            s_state.session_duration_ms = get_le32(payload);
        } else if (payload_len == 0) {
            s_state.session_duration_ms = 0;
        } else {
            printf("[UWB] START_RADAR: bad payload len %u\n\r", payload_len);
            uwb_send_error(port, seq, msg_type, UWB_ERR_INVALID_LENGTH, 0);
            return;
        }

        printf("[UWB] Starting radar, duration=%lums\n\r",
               (unsigned long)s_state.session_duration_ms);
        st = uci_radar_start(s_state.session_handle);
        if (st != UCI_STATUS_OK) {
            printf("[UWB] uci_radar_start failed: 0x%02X\n\r", st);
            uwb_send_error(port, seq, msg_type, UWB_ERR_UCI_COMMAND_FAILED, st);
            return;
        }

        s_state.session_running = true;
        s_state.session_start_tick = HAL_GetTick();
        s_state.session_last_debug_tick = s_state.session_start_tick;
        s_state.session_last_frame_rx_tick = s_state.session_start_tick;
        s_state.session_last_frame_tx_tick = s_state.session_start_tick;
        s_state.session_last_frame_drop_tick = 0U;
        s_state.debug_prev_frame_rx_count = 0U;
        s_state.debug_prev_frame_tx_ok_count = 0U;
        s_state.debug_prev_frame_tx_drop_count = 0U;
        s_state.frame_rx_count = 0;
        s_state.frame_tx_ok_count = 0;
        s_state.frame_tx_drop_count = 0;
        s_state.queue_drop_count = 0;
        s_tx_queue_head = 0U;
        s_tx_queue_tail = 0U;
        s_tx_queue_count = 0U;
        s_tx_queue_high_water = 0U;
        printf("[UWB] Radar started OK\n\r");
        uwb_send_ack(port, seq, msg_type);
        break;

    case UWB_MSG_STOP_RADAR:
        printf("[UWB] STOP_RADAR: sess_init=%d running=%d\n\r",
               s_state.session_initialized, s_state.session_running);
        if (!s_state.session_initialized) {
            uwb_send_error(port, seq, msg_type, UWB_ERR_INVALID_STATE, 0);
            return;
        }

        if (!s_state.session_running) {
            uwb_send_ack(port, seq, msg_type);
            return;
        }

        st = uci_radar_stop(s_state.session_handle);
        if (st != UCI_STATUS_OK) {
            printf("[UWB] uci_radar_stop failed: 0x%02X\n\r", st);
            uwb_send_error(port, seq, msg_type, UWB_ERR_UCI_COMMAND_FAILED, st);
            return;
        }

        s_state.session_running = false;
        printf("[UWB] Radar stopped OK\n\r");
        uwb_send_ack(port, seq, msg_type);
        break;

    default:
        uwb_send_error(port, seq, msg_type, UWB_ERR_INVALID_STATE, msg_type);
        break;
    }
}

void uwb_udp_protocol_tick(void)
{
    if (!s_state.session_running || s_state.session_duration_ms == 0) {
        return;
    }

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - s_state.session_start_tick;
    if ((now - s_state.session_last_debug_tick) >= 1000U) {
        const udp_tx_debug_stats_t *tx_stats = nucleo_udp_get_tx_stats();
        uint32_t rx_delta = s_state.frame_rx_count - s_state.debug_prev_frame_rx_count;
        uint32_t tx_ok_delta = s_state.frame_tx_ok_count - s_state.debug_prev_frame_tx_ok_count;
        uint32_t tx_drop_delta = s_state.frame_tx_drop_count - s_state.debug_prev_frame_tx_drop_count;
        uint32_t last_rx_age = now - s_state.session_last_frame_rx_tick;
        uint32_t last_tx_age = now - s_state.session_last_frame_tx_tick;
        uint32_t last_drop_age = (s_state.session_last_frame_drop_tick == 0U)
                ? 0U
                : (now - s_state.session_last_frame_drop_tick);

        s_state.session_last_debug_tick = now;
        s_state.debug_prev_frame_rx_count = s_state.frame_rx_count;
        s_state.debug_prev_frame_tx_ok_count = s_state.frame_tx_ok_count;
        s_state.debug_prev_frame_tx_drop_count = s_state.frame_tx_drop_count;

        printf("[UWB] stream dbg elapsed=%lums/%lums rx=%lu (+%lu/s) tx_ok=%lu (+%lu/s) tx_drop=%lu (+%lu/s) q_depth=%u q_high=%u q_drop=%lu last_rx_age=%lums last_tx_age=%lums last_drop_age=%lums udp_ok=%lu udp_fail=%lu pbuf_fail=%lu last_err=%ld last_port=%u last_len=%u\n\r",
               (unsigned long)elapsed,
               (unsigned long)s_state.session_duration_ms,
               (unsigned long)s_state.frame_rx_count,
               (unsigned long)rx_delta,
               (unsigned long)s_state.frame_tx_ok_count,
               (unsigned long)tx_ok_delta,
               (unsigned long)s_state.frame_tx_drop_count,
               (unsigned long)tx_drop_delta,
               (unsigned)s_tx_queue_count,
               (unsigned)s_tx_queue_high_water,
               (unsigned long)s_state.queue_drop_count,
               (unsigned long)last_rx_age,
               (unsigned long)last_tx_age,
               (unsigned long)last_drop_age,
               (unsigned long)tx_stats->send_successes,
               (unsigned long)tx_stats->send_failures,
               (unsigned long)tx_stats->pbuf_alloc_failures,
               (long)tx_stats->last_err,
               (unsigned)tx_stats->last_port,
               (unsigned)tx_stats->last_len);
    }

    if (elapsed >= s_state.session_duration_ms) {
        uci_status_t st = uci_radar_stop(s_state.session_handle);
        if (st == UCI_STATUS_OK) {
            s_state.session_running = false;
            printf("[UWB] Session auto-stopped after %lums frame_rx=%lu frame_tx_ok=%lu frame_tx_drop=%lu\n\r",
                   (unsigned long)s_state.session_duration_ms,
                   (unsigned long)s_state.frame_rx_count,
                   (unsigned long)s_state.frame_tx_ok_count,
                   (unsigned long)s_state.frame_tx_drop_count);
        }
    }
}

void uwb_udp_protocol_queue_radar_frame(const uint8_t *raw_payload, uint16_t raw_len)
{
    if (raw_payload == NULL || raw_len < 6U) {
        return;
    }

    s_state.frame_rx_count++;
    s_state.session_last_frame_rx_tick = HAL_GetTick();

    if (raw_len > UWB_RADAR_RAW_MAX_LEN) {
        s_state.frame_tx_drop_count++;
        s_state.queue_drop_count++;
        s_state.session_last_frame_drop_tick = HAL_GetTick();
        printf("[UWB] queue reject raw_len=%u max=%u\n\r",
               (unsigned)raw_len,
               (unsigned)UWB_RADAR_RAW_MAX_LEN);
        return;
    }

    if (s_tx_queue_count >= UWB_TX_QUEUE_DEPTH) {
        s_state.frame_tx_drop_count++;
        s_state.queue_drop_count++;
        s_state.session_last_frame_drop_tick = HAL_GetTick();
        if ((s_state.queue_drop_count <= 8U) || ((s_state.queue_drop_count % 16U) == 0U)) {
            printf("[UWB] tx queue full depth=%u drops=%lu raw_len=%u\n\r",
                   (unsigned)s_tx_queue_count,
                   (unsigned long)s_state.queue_drop_count,
                   (unsigned)raw_len);
        }
        return;
    }

    s_tx_queue[s_tx_queue_tail].raw_len = raw_len;
    memcpy(s_tx_queue[s_tx_queue_tail].raw_payload, raw_payload, raw_len);
    s_tx_queue_tail = (uint8_t)((s_tx_queue_tail + 1U) % UWB_TX_QUEUE_DEPTH);
    s_tx_queue_count++;
    if (s_tx_queue_count > s_tx_queue_high_water) {
        s_tx_queue_high_water = s_tx_queue_count;
    }
}

void uwb_udp_protocol_drain_tx_queue(void)
{
    uint8_t budget = UWB_TX_DRAIN_BUDGET;

    while ((budget > 0U) && (s_tx_queue_count > 0U)) {
        uwb_radar_tx_item_t *item = &s_tx_queue[s_tx_queue_head];
        uwb_udp_protocol_send_radar_frame(item->raw_payload, item->raw_len);
        s_tx_queue_head = (uint8_t)((s_tx_queue_head + 1U) % UWB_TX_QUEUE_DEPTH);
        s_tx_queue_count--;
        budget--;
    }
}

void uwb_udp_protocol_send_radar_frame(const uint8_t *raw_payload, uint16_t raw_len)
{
    /* payload_buf: 8-byte frame header + up to 8192 bytes of raw UCI data */
    static uint8_t payload_buf[8200];
    /* chunk_buf: 4-byte chunk header + one chunk's worth of data, reused per chunk */
    static uint8_t chunk_buf[4 + UWB_MAX_CHUNK_DATA];
    static uint8_t packet_buf[8216];
    static uint32_t frame_count = 0;
    static uint16_t s_frame_id = 0;

    uint16_t payload_len;
    uint32_t session_handle;
    uint8_t status;
    uint8_t radar_data_type;

    if (raw_payload == NULL || raw_len < 6) {
        return;
    }
    if ((uint32_t)raw_len + 8U > sizeof(payload_buf)) {
        return;
    }

    frame_count++;
    session_handle  = get_le32(&raw_payload[0]);
    status          = raw_payload[4];
    radar_data_type = raw_payload[5];

    if (frame_count % 50 == 1) {
        printf("[UWB] CIR frame #%lu type=0x%02X raw_len=%u\n\r",
               (unsigned long)frame_count, radar_data_type, raw_len);
    }
    /* Build the logical payload: 8-byte frame header + raw UCI data */
    put_le32(&payload_buf[0], session_handle);
    payload_buf[4] = status;
    payload_buf[5] = radar_data_type;
    put_le16(&payload_buf[6], raw_len);
    memcpy(&payload_buf[8], raw_payload, raw_len);
    payload_len = (uint16_t)(8U + raw_len);

    if (payload_len <= UWB_MAX_CHUNK_DATA) {
        /* Fits in one datagram — no IP fragmentation needed */
        uint16_t pkt_len = uwb_build_envelope(packet_buf, UWB_MSG_RADAR_FRAME,
                                               0, payload_buf, payload_len);
        if (nucleo_udp_send(UWB_RADAR_FRAME_PORT, packet_buf, pkt_len) == 0U) {
            s_state.frame_tx_ok_count++;
            s_state.session_last_frame_tx_tick = HAL_GetTick();
        } else {
            s_state.frame_tx_drop_count++;
            s_state.session_last_frame_drop_tick = HAL_GetTick();
            printf("[UWB] frame send fail single raw_len=%u pkt_len=%u frame_rx=%lu frame_tx_ok=%lu frame_tx_drop=%lu\n\r",
                   raw_len,
                   pkt_len,
                   (unsigned long)s_state.frame_rx_count,
                   (unsigned long)s_state.frame_tx_ok_count,
                   (unsigned long)s_state.frame_tx_drop_count);
        }
        return;
    }

    /* Large payload: split into MTU-safe RADAR_FRAME_CHUNK packets.
     * Chunk payload: frame_id(2B LE) | chunk_idx(1B) | total_chunks(1B) | data(N B)
     * Each UDP datagram stays below 1472 bytes (Ethernet MTU limit). */
    uint16_t total_chunks = (uint16_t)((payload_len + UWB_MAX_CHUNK_DATA - 1U) / UWB_MAX_CHUNK_DATA);
    uint16_t offset = 0;
    bool frame_send_ok = true;

    if (++s_frame_id == 0) s_frame_id = 1;

    if (frame_count % 64U == 1U) {
        printf("[UWB] frame chunking raw_len=%u payload_len=%u frame_id=%u total_chunks=%u\n\r",
               raw_len,
               payload_len,
               (unsigned)s_frame_id,
               (unsigned)total_chunks);
    }

    for (uint16_t i = 0; i < total_chunks; i++) {
        uint16_t chunk_data_len = (uint16_t)(
            (offset + UWB_MAX_CHUNK_DATA <= payload_len)
                ? UWB_MAX_CHUNK_DATA
                : (payload_len - offset));

        chunk_buf[0] = (uint8_t)(s_frame_id & 0xFF);
        chunk_buf[1] = (uint8_t)(s_frame_id >> 8);
        chunk_buf[2] = (uint8_t)i;
        chunk_buf[3] = (uint8_t)total_chunks;
        memcpy(&chunk_buf[4], &payload_buf[offset], chunk_data_len);

        uint16_t pkt_len = uwb_build_envelope(packet_buf, UWB_MSG_RADAR_FRAME_CHUNK,
                                               0, chunk_buf,
                                               (uint16_t)(4U + chunk_data_len));
        if (nucleo_udp_send(UWB_RADAR_FRAME_PORT, packet_buf, pkt_len) != 0U) {
            frame_send_ok = false;
            printf("[UWB] chunk send fail frame_id=%u chunk=%u/%u chunk_data_len=%u pkt_len=%u raw_len=%u\n\r",
                   (unsigned)s_frame_id,
                   (unsigned)(i + 1U),
                   (unsigned)total_chunks,
                   (unsigned)chunk_data_len,
                   (unsigned)pkt_len,
                   (unsigned)raw_len);
        }
        offset += chunk_data_len;
    }

    if (frame_send_ok) {
        s_state.frame_tx_ok_count++;
        s_state.session_last_frame_tx_tick = HAL_GetTick();
    } else {
        s_state.frame_tx_drop_count++;
        s_state.session_last_frame_drop_tick = HAL_GetTick();
        if ((s_state.frame_tx_drop_count <= 8U) || ((s_state.frame_tx_drop_count % 16U) == 0U)) {
            const udp_tx_debug_stats_t *tx_stats = nucleo_udp_get_tx_stats();
            printf("[UWB] frame drop summary frame_id=%u raw_len=%u total_chunks=%u drops=%lu udp_ok=%lu udp_fail=%lu pbuf_fail=%lu last_err=%ld last_len=%u\n\r",
                   (unsigned)s_frame_id,
                   (unsigned)raw_len,
                   (unsigned)total_chunks,
                   (unsigned long)s_state.frame_tx_drop_count,
                   (unsigned long)tx_stats->send_successes,
                   (unsigned long)tx_stats->send_failures,
                   (unsigned long)tx_stats->pbuf_alloc_failures,
                   (long)tx_stats->last_err,
                   (unsigned)tx_stats->last_len);
        }
    }
}

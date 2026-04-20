#include "uwb_udp_protocol.h"

#include <string.h>
#include <stdio.h>

#include "udp_server.h"
#include "uci/uci_radar.h"
#include "uci/uci_types.h"

typedef struct {
    uci_radar_params_t current_cfg;
    uint32_t session_handle;
    bool session_initialized;
    bool session_config_valid;
    bool session_running;
} uwb_protocol_state_t;

static uwb_protocol_state_t s_state;

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

    payload[0] = acked_msg_type;
    put_le16(&payload[1], seq);

    uint16_t packet_len = uwb_build_envelope(packet, UWB_MSG_ACK, seq, payload, sizeof(payload));
    nucleo_udp_send(reply_port, packet, packet_len);
}

static void uwb_send_error(uint16_t reply_port, uint16_t seq,
                            uint8_t failed_msg_type, uint16_t error_code, uint8_t detail)
{
    uint8_t payload[6];
    uint8_t packet[20];

    payload[0] = failed_msg_type;
    put_le16(&payload[1], seq);
    put_le16(&payload[3], error_code);
    payload[5] = detail;

    uint16_t packet_len = uwb_build_envelope(packet, UWB_MSG_ERROR, seq, payload, sizeof(payload));
    nucleo_udp_send(reply_port, packet, packet_len);
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

    if (data == NULL || len < 8) {
        return;
    }

    if (data[0] != UWB_UDP_MAGIC_0 || data[1] != UWB_UDP_MAGIC_1) {
        uwb_send_error(port, 0, 0, UWB_ERR_INVALID_MAGIC, 0);
        return;
    }

    if (data[2] != UWB_UDP_PROTOCOL_VERSION) {
        uwb_send_error(port, get_le16(&data[4]), data[3], UWB_ERR_INVALID_VERSION, data[2]);
        return;
    }

    msg_type = data[3];
    seq = get_le16(&data[4]);
    payload_len = get_le16(&data[6]);

    if (len != (uint16_t)(8 + payload_len)) {
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

        st = uwb_parse_tlvs_into_cfg(payload, payload_len, &new_cfg, &seen_mask, &bad_tag);
        if (st != UCI_STATUS_OK) {
            uwb_send_error(port, seq, msg_type,
                           (st == UCI_STATUS_INVALID_PARAM) ? UWB_ERR_UNSUPPORTED_TAG : UWB_ERR_INVALID_TLV,
                           (uint8_t)(bad_tag & 0xFF));
            return;
        }

        if (seen_mask != uwb_required_mask()) {
            uwb_send_error(port, seq, msg_type, UWB_ERR_MISSING_REQUIRED_FIELD, 0);
            return;
        }

        s_state.current_cfg = new_cfg;
        st = uwb_apply_config(false);
        if (st != UCI_STATUS_OK) {
            uwb_send_error(port, seq, msg_type, UWB_ERR_UCI_COMMAND_FAILED, st);
            return;
        }

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
        if (!s_state.session_config_valid || !s_state.session_initialized) {
            uwb_send_error(port, seq, msg_type, UWB_ERR_INVALID_STATE, 0);
            return;
        }

        if (s_state.session_running) {
            uwb_send_ack(port, seq, msg_type);
            return;
        }

        st = uci_radar_start(s_state.session_handle);
        if (st != UCI_STATUS_OK) {
            uwb_send_error(port, seq, msg_type, UWB_ERR_UCI_COMMAND_FAILED, st);
            return;
        }

        s_state.session_running = true;
        uwb_send_ack(port, seq, msg_type);
        break;

    case UWB_MSG_STOP_RADAR:
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
            uwb_send_error(port, seq, msg_type, UWB_ERR_UCI_COMMAND_FAILED, st);
            return;
        }

        s_state.session_running = false;
        uwb_send_ack(port, seq, msg_type);
        break;

    default:
        uwb_send_error(port, seq, msg_type, UWB_ERR_INVALID_STATE, msg_type);
        break;
    }
}

void uwb_udp_protocol_send_radar_frame(const uint8_t *raw_payload, uint16_t raw_len)
{
    static uint8_t payload_buf[8200];
    static uint8_t packet_buf[8216];
    uint16_t payload_len;
    uint16_t packet_len;
    uint32_t session_handle;
    uint8_t status;
    uint8_t radar_data_type;

    if (raw_payload == NULL || raw_len < 6) {
        return;
    }

    if ((uint32_t)raw_len + 8U > sizeof(payload_buf)) {
        return;
    }

    session_handle = get_le32(&raw_payload[0]);
    status = raw_payload[4];
    radar_data_type = raw_payload[5];

    put_le32(&payload_buf[0], session_handle);
    payload_buf[4] = status;
    payload_buf[5] = radar_data_type;
    put_le16(&payload_buf[6], raw_len);
    memcpy(&payload_buf[8], raw_payload, raw_len);
    payload_len = (uint16_t)(8 + raw_len);

    packet_len = uwb_build_envelope(packet_buf, UWB_MSG_RADAR_FRAME, 0, payload_buf, payload_len);
    nucleo_udp_send(UWB_RADAR_FRAME_PORT, packet_buf, packet_len);
}

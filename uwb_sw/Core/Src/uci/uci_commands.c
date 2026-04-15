/*
 * uci_commands.c
 *
 * High-level UCI command builder implementation.
 * Each function builds a command payload, calls uci_core_send_command(),
 * and parses the response.
 */

#include "uci/uci_commands.h"
#include "uci/uci_core.h"
#include "uci/uci_defs.h"
#include <string.h>

/* ========================================================================== */
/*  Little-Endian Encoding Helpers                                            */
/* ========================================================================== */

static inline void put_le16(uint8_t *buf, uint16_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)(val >> 8);
}

static inline void put_le32(uint8_t *buf, uint32_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

static inline uint16_t get_le16(const uint8_t *buf)
{
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static inline uint32_t get_le32(const uint8_t *buf)
{
    return (uint32_t)buf[0] |
           ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) |
           ((uint32_t)buf[3] << 24);
}

/* ========================================================================== */
/*  TLV Helpers                                                               */
/* ========================================================================== */

void uci_tlv_build_u8(uci_tlv_t *tlv, uint16_t tag, uint8_t value)
{
    tlv->tag = tag;
    tlv->len = 1;
    tlv->value[0] = value;
}

void uci_tlv_build_u16(uci_tlv_t *tlv, uint16_t tag, uint16_t value)
{
    tlv->tag = tag;
    tlv->len = 2;
    put_le16(tlv->value, value);
}

void uci_tlv_build_u32(uci_tlv_t *tlv, uint16_t tag, uint32_t value)
{
    tlv->tag = tag;
    tlv->len = 4;
    put_le32(tlv->value, value);
}

void uci_tlv_build_array(uci_tlv_t *tlv, uint16_t tag, const uint8_t *data, uint8_t len)
{
    tlv->tag = tag;
    tlv->len = len;
    if (data != NULL && len > 0) {
        memcpy(tlv->value, data, len);
    }
}

/*
 * Serialize TLVs into a byte buffer.
 *
 * Standard (tag < 0x100): [tag:1] [len:1] [value:len]
 * NXP extended (tag >= 0xE300): [tag_hi:1] [tag_lo:1] [len:1] [value:len]
 *
 * Returns total bytes written.
 */
uint16_t uci_tlv_serialize(const uci_tlv_t *tlvs, uint8_t num_tlvs, uint8_t *buf)
{
    uint16_t offset = 0;

    for (uint8_t i = 0; i < num_tlvs; i++) {
        if (tlvs[i].tag >= 0xE300) {
            /* 16-bit extended tag */
            buf[offset++] = (uint8_t)(tlvs[i].tag >> 8);
            buf[offset++] = (uint8_t)(tlvs[i].tag & 0xFF);
        } else {
            /* 8-bit standard tag */
            buf[offset++] = (uint8_t)(tlvs[i].tag & 0xFF);
        }
        buf[offset++] = tlvs[i].len;
        if (tlvs[i].len > 0) {
            memcpy(&buf[offset], tlvs[i].value, tlvs[i].len);
            offset += tlvs[i].len;
        }
    }
    return offset;
}

/*
 * Deserialize a byte buffer into TLVs.
 * Handles both 8-bit and 16-bit extended tags (0xE3xx / 0xE4xx prefix).
 * Returns number of TLVs parsed.
 */
uint8_t uci_tlv_deserialize(const uint8_t *buf, uint16_t len,
                             uci_tlv_t *tlvs, uint8_t max_tlvs)
{
    uint16_t offset = 0;
    uint8_t count = 0;

    while (offset < len && count < max_tlvs) {
        uint16_t tag;

        /* Detect 16-bit extended tag prefix (0xE3 or 0xE4) */
        if (offset + 1 < len &&
            (buf[offset] == 0xE3 || buf[offset] == 0xE4)) {
            tag = ((uint16_t)buf[offset] << 8) | (uint16_t)buf[offset + 1];
            offset += 2;
        } else {
            tag = (uint16_t)buf[offset];
            offset += 1;
        }

        if (offset >= len) break;
        uint8_t vlen = buf[offset++];

        if (offset + vlen > len) break;

        tlvs[count].tag = tag;
        tlvs[count].len = vlen;
        if (vlen > 0 && vlen <= UCI_MAX_TLV_VALUE_LEN) {
            memcpy(tlvs[count].value, &buf[offset], vlen);
        }
        offset += vlen;
        count++;
    }
    return count;
}

/* ========================================================================== */
/*  Core Group (GID 0x00)                                                     */
/* ========================================================================== */

uci_status_t uci_cmd_device_reset(void)
{
    /* CORE_DEVICE_RESET_CMD: payload = [reset_config: 1 byte] */
    uint8_t payload[1] = { UCI_RESET_CONFIG_UWBS_RESET };
    uint8_t rsp[4];
    uint16_t rsp_len = sizeof(rsp);

    return uci_core_send_command(UCI_GID_CORE, UCI_OID_CORE_DEVICE_RESET,
                                 payload, 1, rsp, &rsp_len,
                                 UCI_CMD_RESPONSE_TIMEOUT_MS);
}

uci_status_t uci_cmd_get_device_info(uci_device_info_t *info)
{
    if (info == NULL) return UCI_STATUS_INVALID_PARAM;

    uint8_t rsp[256];
    uint16_t rsp_len = sizeof(rsp);

    uci_status_t st = uci_core_send_command(UCI_GID_CORE, UCI_OID_CORE_GET_DEVICE_INFO,
                                             NULL, 0, rsp, &rsp_len,
                                             UCI_CMD_RESPONSE_TIMEOUT_MS);
    if (st != UCI_STATUS_OK) return st;

    /* Parse CORE_GET_DEVICE_INFO_RSP:
     *   [status:1] [uci_ver:2] [mac_ver:2] [phy_ver:2] [uci_test_ver:2]
     *   [vendor_spec_len:1] [vendor_spec_info:N] */
    if (rsp_len < 10) return UCI_STATUS_FAILED;

    info->status           = rsp[0];
    info->uci_version      = get_le16(&rsp[1]);
    info->mac_version      = get_le16(&rsp[3]);
    info->phy_version      = get_le16(&rsp[5]);
    info->uci_test_version = get_le16(&rsp[7]);
    info->vendor_spec_len  = rsp[9];

    if (info->vendor_spec_len > 0) {
        uint8_t copy_len = info->vendor_spec_len;
        if (copy_len > sizeof(info->vendor_spec_info)) {
            copy_len = sizeof(info->vendor_spec_info);
        }
        if (rsp_len >= 10 + copy_len) {
            memcpy(info->vendor_spec_info, &rsp[10], copy_len);
        }
    }

    return UCI_STATUS_OK;
}

uci_status_t uci_cmd_get_caps_info(uint8_t *caps_tlv_buf, uint16_t *caps_len)
{
    if (caps_tlv_buf == NULL || caps_len == NULL) return UCI_STATUS_INVALID_PARAM;

    uint8_t rsp[512];
    uint16_t rsp_len = sizeof(rsp);

    uci_status_t st = uci_core_send_command(UCI_GID_CORE, UCI_OID_CORE_GET_CAPS_INFO,
                                             NULL, 0, rsp, &rsp_len,
                                             UCI_CMD_RESPONSE_TIMEOUT_MS);
    if (st != UCI_STATUS_OK) return st;

    /* CORE_GET_CAPS_INFO_RSP: [status:1] [num_params:1] [TLV data...] */
    if (rsp_len < 2) return UCI_STATUS_FAILED;

    uint16_t tlv_len = rsp_len - 2;
    if (tlv_len > *caps_len) tlv_len = *caps_len;
    memcpy(caps_tlv_buf, &rsp[2], tlv_len);
    *caps_len = tlv_len;

    return UCI_STATUS_OK;
}

uci_status_t uci_cmd_set_config(const uci_tlv_t *tlvs, uint8_t num_tlvs)
{
    if (tlvs == NULL || num_tlvs == 0) return UCI_STATUS_INVALID_PARAM;

    uint8_t payload[UCI_MAX_PAYLOAD_SIZE];
    uint16_t offset = 0;

    /* [num_params:1] [TLV data...] */
    payload[offset++] = num_tlvs;
    offset += uci_tlv_serialize(tlvs, num_tlvs, &payload[offset]);

    uint8_t rsp[64];
    uint16_t rsp_len = sizeof(rsp);

    return uci_core_send_command(UCI_GID_CORE, UCI_OID_CORE_SET_CONFIG,
                                 payload, offset, rsp, &rsp_len,
                                 UCI_CMD_RESPONSE_TIMEOUT_MS);
}

uci_status_t uci_cmd_get_config(const uint8_t *param_ids, uint8_t num_params,
                                 uci_tlv_t *out_tlvs, uint8_t *out_num_tlvs)
{
    if (param_ids == NULL || num_params == 0 || out_tlvs == NULL || out_num_tlvs == NULL) {
        return UCI_STATUS_INVALID_PARAM;
    }

    uint8_t payload[UCI_MAX_PAYLOAD_SIZE];
    uint16_t offset = 0;

    /* [num_params:1] [param_id:1 * N] */
    payload[offset++] = num_params;
    memcpy(&payload[offset], param_ids, num_params);
    offset += num_params;

    uint8_t rsp[512];
    uint16_t rsp_len = sizeof(rsp);

    uci_status_t st = uci_core_send_command(UCI_GID_CORE, UCI_OID_CORE_GET_CONFIG,
                                             payload, offset, rsp, &rsp_len,
                                             UCI_CMD_RESPONSE_TIMEOUT_MS);
    if (st != UCI_STATUS_OK) return st;

    /* CORE_GET_CONFIG_RSP: [status:1] [num_params:1] [TLV data...] */
    if (rsp_len < 2) return UCI_STATUS_FAILED;

    *out_num_tlvs = uci_tlv_deserialize(&rsp[2], rsp_len - 2, out_tlvs, *out_num_tlvs);
    return UCI_STATUS_OK;
}

/* ========================================================================== */
/*  Session Config Group (GID 0x01)                                           */
/* ========================================================================== */

uci_status_t uci_cmd_session_init(uint32_t session_id, uint8_t session_type,
                                   uint32_t *out_session_handle)
{
    /* SESSION_INIT_CMD: [session_id:4 LE] [session_type:1] */
    uint8_t payload[5];
    put_le32(&payload[0], session_id);
    payload[4] = session_type;

    uint8_t rsp[16];
    uint16_t rsp_len = sizeof(rsp);

    uci_status_t st = uci_core_send_command(UCI_GID_SESSION_CONFIG, UCI_OID_SESSION_INIT,
                                             payload, 5, rsp, &rsp_len,
                                             UCI_CMD_RESPONSE_TIMEOUT_MS);
    if (st != UCI_STATUS_OK) return st;

    /* SESSION_INIT_RSP: [status:1] [session_handle:4 LE] (if present) */
    if (out_session_handle != NULL) {
        if (rsp_len >= 5) {
            *out_session_handle = get_le32(&rsp[1]);
        } else {
            /* Some FW versions return session_id as the handle */
            *out_session_handle = session_id;
        }
    }

    return UCI_STATUS_OK;
}

uci_status_t uci_cmd_session_deinit(uint32_t session_handle)
{
    /* SESSION_DEINIT_CMD: [session_handle:4 LE] */
    uint8_t payload[4];
    put_le32(payload, session_handle);

    uint8_t rsp[8];
    uint16_t rsp_len = sizeof(rsp);

    return uci_core_send_command(UCI_GID_SESSION_CONFIG, UCI_OID_SESSION_DEINIT,
                                 payload, 4, rsp, &rsp_len,
                                 UCI_CMD_RESPONSE_TIMEOUT_MS);
}

uci_status_t uci_cmd_session_set_app_config(uint32_t session_handle,
                                             const uci_tlv_t *tlvs, uint8_t num_tlvs)
{
    if (tlvs == NULL || num_tlvs == 0) return UCI_STATUS_INVALID_PARAM;

    uint8_t payload[UCI_MAX_EXT_PAYLOAD_SIZE];
    uint16_t offset = 0;

    /* [session_handle:4 LE] [num_params:1] [TLV data...] */
    put_le32(&payload[offset], session_handle);
    offset += 4;
    payload[offset++] = num_tlvs;
    offset += uci_tlv_serialize(tlvs, num_tlvs, &payload[offset]);

    uint8_t rsp[64];
    uint16_t rsp_len = sizeof(rsp);

    return uci_core_send_command(UCI_GID_SESSION_CONFIG, UCI_OID_SESSION_SET_APP_CONFIG,
                                 payload, offset, rsp, &rsp_len,
                                 UCI_CMD_RESPONSE_TIMEOUT_MS);
}

uci_status_t uci_cmd_session_get_app_config(uint32_t session_handle,
                                             const uint8_t *param_ids, uint8_t num_params,
                                             uci_tlv_t *out_tlvs, uint8_t *out_num_tlvs)
{
    if (param_ids == NULL || out_tlvs == NULL || out_num_tlvs == NULL) {
        return UCI_STATUS_INVALID_PARAM;
    }

    uint8_t payload[UCI_MAX_PAYLOAD_SIZE];
    uint16_t offset = 0;

    /* [session_handle:4 LE] [num_params:1] [param_id:1 * N] */
    put_le32(&payload[offset], session_handle);
    offset += 4;
    payload[offset++] = num_params;
    memcpy(&payload[offset], param_ids, num_params);
    offset += num_params;

    uint8_t rsp[512];
    uint16_t rsp_len = sizeof(rsp);

    uci_status_t st = uci_core_send_command(UCI_GID_SESSION_CONFIG,
                                             UCI_OID_SESSION_GET_APP_CONFIG,
                                             payload, offset, rsp, &rsp_len,
                                             UCI_CMD_RESPONSE_TIMEOUT_MS);
    if (st != UCI_STATUS_OK) return st;

    /* RSP: [status:1] [num_params:1] [TLV data...] */
    if (rsp_len < 2) return UCI_STATUS_FAILED;

    *out_num_tlvs = uci_tlv_deserialize(&rsp[2], rsp_len - 2, out_tlvs, *out_num_tlvs);
    return UCI_STATUS_OK;
}

uci_status_t uci_cmd_session_get_count(uint8_t *count)
{
    if (count == NULL) return UCI_STATUS_INVALID_PARAM;

    uint8_t rsp[8];
    uint16_t rsp_len = sizeof(rsp);

    uci_status_t st = uci_core_send_command(UCI_GID_SESSION_CONFIG,
                                             UCI_OID_SESSION_GET_COUNT,
                                             NULL, 0, rsp, &rsp_len,
                                             UCI_CMD_RESPONSE_TIMEOUT_MS);
    if (st != UCI_STATUS_OK) return st;

    /* RSP: [status:1] [count:1] */
    if (rsp_len >= 2) {
        *count = rsp[1];
    }
    return UCI_STATUS_OK;
}

uci_status_t uci_cmd_session_get_state(uint32_t session_handle, uint8_t *state)
{
    if (state == NULL) return UCI_STATUS_INVALID_PARAM;

    uint8_t payload[4];
    put_le32(payload, session_handle);

    uint8_t rsp[8];
    uint16_t rsp_len = sizeof(rsp);

    uci_status_t st = uci_core_send_command(UCI_GID_SESSION_CONFIG,
                                             UCI_OID_SESSION_GET_STATE,
                                             payload, 4, rsp, &rsp_len,
                                             UCI_CMD_RESPONSE_TIMEOUT_MS);
    if (st != UCI_STATUS_OK) return st;

    /* RSP: [status:1] [session_state:1] */
    if (rsp_len >= 2) {
        *state = rsp[1];
    }
    return UCI_STATUS_OK;
}

uci_status_t uci_cmd_session_update_multicast_list(uint32_t session_handle,
                                                    uint8_t action,
                                                    uint8_t num_controlees,
                                                    const uint16_t *controlee_addrs)
{
    if (controlee_addrs == NULL || num_controlees == 0) return UCI_STATUS_INVALID_PARAM;

    uint8_t payload[UCI_MAX_PAYLOAD_SIZE];
    uint16_t offset = 0;

    /* [session_handle:4] [action:1] [num_controlees:1] [addr:2 * N] */
    put_le32(&payload[offset], session_handle);
    offset += 4;
    payload[offset++] = action;
    payload[offset++] = num_controlees;
    for (uint8_t i = 0; i < num_controlees; i++) {
        put_le16(&payload[offset], controlee_addrs[i]);
        offset += 2;
    }

    uint8_t rsp[16];
    uint16_t rsp_len = sizeof(rsp);

    return uci_core_send_command(UCI_GID_SESSION_CONFIG,
                                 UCI_OID_SESSION_UPDATE_MCAST_LIST,
                                 payload, offset, rsp, &rsp_len,
                                 UCI_CMD_RESPONSE_TIMEOUT_MS);
}

/* ========================================================================== */
/*  Session Control Group (GID 0x02)                                          */
/* ========================================================================== */

uci_status_t uci_cmd_session_start(uint32_t session_handle)
{
    uint8_t payload[4];
    put_le32(payload, session_handle);

    uint8_t rsp[8];
    uint16_t rsp_len = sizeof(rsp);

    return uci_core_send_command(UCI_GID_SESSION_CONTROL, UCI_OID_SESSION_START,
                                 payload, 4, rsp, &rsp_len,
                                 UCI_CMD_RESPONSE_TIMEOUT_MS);
}

uci_status_t uci_cmd_session_stop(uint32_t session_handle)
{
    uint8_t payload[4];
    put_le32(payload, session_handle);

    uint8_t rsp[8];
    uint16_t rsp_len = sizeof(rsp);

    return uci_core_send_command(UCI_GID_SESSION_CONTROL, UCI_OID_SESSION_STOP,
                                 payload, 4, rsp, &rsp_len,
                                 UCI_CMD_RESPONSE_TIMEOUT_MS);
}

uci_status_t uci_cmd_session_get_ranging_count(uint32_t session_handle, uint32_t *count)
{
    if (count == NULL) return UCI_STATUS_INVALID_PARAM;

    uint8_t payload[4];
    put_le32(payload, session_handle);

    uint8_t rsp[16];
    uint16_t rsp_len = sizeof(rsp);

    uci_status_t st = uci_core_send_command(UCI_GID_SESSION_CONTROL,
                                             UCI_OID_SESSION_GET_RANGING_COUNT,
                                             payload, 4, rsp, &rsp_len,
                                             UCI_CMD_RESPONSE_TIMEOUT_MS);
    if (st != UCI_STATUS_OK) return st;

    /* RSP: [status:1] [count:4 LE] */
    if (rsp_len >= 5) {
        *count = get_le32(&rsp[1]);
    }
    return UCI_STATUS_OK;
}

/* ========================================================================== */
/*  Convenience — Ranging (secondary use case)                                */
/* ========================================================================== */

uci_status_t uci_cmd_set_ranging_params(uint32_t session_handle,
                                         const uci_ranging_params_t *params)
{
    if (params == NULL) return UCI_STATUS_INVALID_PARAM;

    uci_tlv_t tlvs[16];
    uint8_t n = 0;

    uci_tlv_build_u8(&tlvs[n++],  UCI_PARAM_DEVICE_TYPE,          params->device_type);
    uci_tlv_build_u8(&tlvs[n++],  UCI_PARAM_DEVICE_ROLE,          params->device_role);
    uci_tlv_build_u8(&tlvs[n++],  UCI_PARAM_RANGING_ROUND_USAGE,  params->ranging_round_usage);
    uci_tlv_build_u8(&tlvs[n++],  UCI_PARAM_MULTI_NODE_MODE,      params->multi_node_mode);
    uci_tlv_build_u8(&tlvs[n++],  UCI_PARAM_CHANNEL_NUMBER,       params->channel_number);
    uci_tlv_build_u8(&tlvs[n++],  UCI_PARAM_NUM_OF_CONTROLEES,    params->number_of_controlees);
    uci_tlv_build_u16(&tlvs[n++], UCI_PARAM_DEVICE_MAC_ADDRESS,   params->device_mac_address);
    uci_tlv_build_u16(&tlvs[n++], UCI_PARAM_SLOT_DURATION,        params->slot_duration_rstu);
    uci_tlv_build_u32(&tlvs[n++], UCI_PARAM_RANGING_DURATION,     params->ranging_duration_ms);
    uci_tlv_build_u8(&tlvs[n++],  UCI_PARAM_STS_CONFIG,           params->sts_config);
    uci_tlv_build_u8(&tlvs[n++],  UCI_PARAM_AOA_RESULT_REQ,       params->aoa_result_req);
    uci_tlv_build_u8(&tlvs[n++],  UCI_PARAM_RFRAME_CONFIG,        params->rframe_config);
    uci_tlv_build_u8(&tlvs[n++],  UCI_PARAM_PREAMBLE_CODE_INDEX,  params->preamble_code_index);
    uci_tlv_build_u8(&tlvs[n++],  UCI_PARAM_SFD_ID,               params->sfd_id);
    uci_tlv_build_u8(&tlvs[n++],  UCI_PARAM_SLOTS_PER_RR,         params->slots_per_rr);
    uci_tlv_build_u8(&tlvs[n++],  UCI_PARAM_SCHEDULE_MODE,        params->schedule_mode);

    /* Set standard app config first */
    uci_status_t st = uci_cmd_session_set_app_config(session_handle, tlvs, n);
    if (st != UCI_STATUS_OK) return st;

    /* Set destination addresses via a separate call if needed */
    if (params->number_of_controlees > 0) {
        uci_tlv_t dst_tlv;
        uint8_t addr_buf[16]; /* Up to 8 addresses * 2 bytes */
        uint8_t addr_len = 0;
        for (uint8_t i = 0; i < params->number_of_controlees && i < 8; i++) {
            put_le16(&addr_buf[addr_len], params->dst_mac_address[i]);
            addr_len += 2;
        }
        uci_tlv_build_array(&dst_tlv, UCI_PARAM_DST_MAC_ADDRESS, addr_buf, addr_len);
        st = uci_cmd_session_set_app_config(session_handle, &dst_tlv, 1);
    }

    return st;
}

uci_status_t uci_parse_session_info_ntf(const uint8_t *payload, uint16_t len,
                                         uci_session_info_ntf_t *result)
{
    if (payload == NULL || result == NULL) return UCI_STATUS_INVALID_PARAM;

    /* SESSION_INFO_NTF payload:
     *   [sequence_number:4] [session_handle:4] [rcr_indicator:1]
     *   [current_ranging_interval:4] [ranging_measurement_type:1]
     *   [mac_addressing_mode:1] [num_measurements:1] [measurements...] */
    if (len < 16) return UCI_STATUS_FAILED;

    memset(result, 0, sizeof(*result));

    uint16_t off = 0;
    result->sequence_number         = get_le32(&payload[off]); off += 4;
    result->session_handle          = get_le32(&payload[off]); off += 4;
    off += 1; /* rcr_indicator */
    off += 4; /* current_ranging_interval */
    result->ranging_measurement_type = payload[off++];
    result->mac_addressing_mode      = payload[off++];
    result->num_measurements         = payload[off++];

    /* Parse each measurement (short MAC address mode: 2-byte addr) */
    for (uint8_t i = 0; i < result->num_measurements && i < 8; i++) {
        if (off + 12 > len) break;

        uci_ranging_measurement_t *m = &result->measurements[i];
        m->mac_address          = get_le16(&payload[off]); off += 2;
        m->status               = payload[off++];
        m->nlos                 = payload[off++];
        m->distance_cm          = get_le16(&payload[off]); off += 2;
        m->aoa_azimuth_deg_q7   = (int16_t)get_le16(&payload[off]); off += 2;
        m->aoa_elevation_deg_q7 = (int16_t)get_le16(&payload[off]); off += 2;
        m->aoa_azimuth_fom      = payload[off++];
        m->aoa_elevation_fom    = payload[off++];

        /* NXP extended fields (if present) */
        if (off + 4 <= len) {
            m->rssi_dbm = (int16_t)get_le16(&payload[off]); off += 2;
            m->pdoa_q7  = (int16_t)get_le16(&payload[off]); off += 2;
        }
    }

    return UCI_STATUS_OK;
}

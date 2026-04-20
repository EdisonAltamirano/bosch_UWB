/*
 * uci_sr250.c
 *
 * NXP SR250-specific vendor extension implementation.
 *
 * Initialization sequence per NXP SR250 UCI Specification v2.0.23 §6.1:
 *   HW reset → wait DEVICE_STATUS_NTF(READY) → CORE_DEVICE_INIT → wait READY
 *   → antenna config → calibration → get device info
 *
 * Antenna configuration uses CORE_SET_CONFIG with NXP extended tags (0xE4/XX).
 * Vendor app config uses GID 0x0F.
 * Calibration uses GID 0x0F OID 0x20/0x21/0x22.
 */

#include "uci/uci_sr250.h"
#include "uci/uci_core.h"
#include "uci/uci_commands.h"
#include "uci/uci_transport.h"
#include "uci/uci_defs.h"
#include "main.h"
#include <string.h>

/* ========================================================================== */
/*  Little-Endian Helpers (local)                                             */
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

/* ========================================================================== */
/*  Device Initialization                                                     */
/* ========================================================================== */

uci_status_t uci_sr250_device_reset(void)
{
    /* CORE_DEVICE_RESET (GID 0x00, OID 0x00) */
	uint8_t payload[1] = { 0x00 };
    uint8_t rsp[8];
    uint16_t rsp_len = sizeof(rsp);

    return uci_core_send_command(UCI_GID_CORE, UCI_OID_CORE_DEVICE_RESET,
                                 payload, 1, rsp, &rsp_len,
                                 UCI_CMD_RESPONSE_TIMEOUT_MS);
}

uci_status_t uci_sr250_device_init(uint8_t major_version, uint8_t minor_version)
{
    /* CORE_DEVICE_INIT_CMD (GID 0x0E, OID 0x00):
     *   Payload: [major_version:1] [minor_version:1] */
    uint8_t payload[2] = { major_version, minor_version };
    uint8_t rsp[8];
    uint16_t rsp_len = sizeof(rsp);

    return uci_core_send_command(UCI_GID_VENDOR_E, UCI_OID_NXP_CORE_DEVICE_INIT,
                                 payload, 2, rsp, &rsp_len,
                                 UCI_CMD_RESPONSE_TIMEOUT_MS);
}

uci_status_t uci_sr250_get_device_info(void)
{
    /* CORE_DEVICE_RESET (GID 0x00, OID 0x00) */
	uint8_t payload[1] = { 0x00 };
    uint8_t rsp[255];
    uint16_t rsp_len = sizeof(rsp);

    return uci_core_send_command(UCI_GID_CORE, UCI_OID_CORE_GET_DEVICE_INFO,
                                 NULL, 0, rsp, &rsp_len,
                                 UCI_CMD_RESPONSE_TIMEOUT_MS);
}

/* ========================================================================== */
/*  Antenna Configuration                                                     */
/* ========================================================================== */

/*
 * NXP extended device config tags use a 2-byte tag format: [0xE4] [sub_tag].
 * These are sent via CORE_SET_CONFIG (GID 0x00, OID 0x04).
 *
 * RX antenna define (tag 0xE4/0x60):
 *   Value per antenna: [antenna_id:1] [rx_port:1] [gpio_mask:1] [gpio_state:1]
 *
 * TX antenna define (tag 0xE4/0x63):
 *   Value per antenna: [antenna_id:1] [tx_port:1] [gpio_mask:1] [gpio_state:1]
 *
 * RX antenna pair define (tag 0xE4/0x62):
 *   Value per pair: [pair_id:1] [antenna_rxc:1] [antenna_rxb:1] [antenna_rxa:1]
 */

uci_status_t uci_sr250_set_rx_antenna_defs(const uci_rx_antenna_def_t *antennas, uint8_t count)
{
    if (antennas == NULL || count == 0) return UCI_STATUS_INVALID_PARAM;

    /* Build TLV: tag=0xE460, value = count * 4 bytes */
    uci_tlv_t tlv;
    tlv.tag = ((uint16_t)NXP_CFG_EXT_TAG_PREFIX << 8) | NXP_CFG_ANTENNA_RX_IDX_DEFINE;
    tlv.len = (count * 6) + 1;

    tlv.value[0] = count;
    for (uint8_t i = 0; i < count && i * 6 < UCI_MAX_TLV_VALUE_LEN; i++) {
        uint8_t off = (i * 6) + 1;
        tlv.value[off + 0] = antennas[i].antenna_id;
        tlv.value[off + 1] = antennas[i].rx_port;
        tlv.value[off + 2] = antennas[i].gpio_mask_lsb;
        tlv.value[off + 3] = antennas[i].gpio_mask_msb;
        tlv.value[off + 4] = antennas[i].gpio_state_lsb;
        tlv.value[off + 5] = antennas[i].gpio_state_msb;
    }

    return uci_cmd_set_config(&tlv, 1);
}

uci_status_t uci_sr250_set_tx_antenna_defs(const uci_tx_antenna_def_t *antennas, uint8_t count)
{
    if (antennas == NULL || count == 0) return UCI_STATUS_INVALID_PARAM;

    uci_tlv_t tlv;
    tlv.tag = ((uint16_t)NXP_CFG_EXT_TAG_PREFIX << 8) | NXP_CFG_ANTENNA_TX_IDX_DEFINE;
    tlv.len = (count * 6) + 1;

    tlv.value[0] = count;
    for (uint8_t i = 0; i < count && i * 6 < UCI_MAX_TLV_VALUE_LEN; i++) {
        uint8_t off = (i * 6) + 1;
        tlv.value[off + 0] = antennas[i].antenna_id;
        tlv.value[off + 1] = antennas[i].tx_port;
        tlv.value[off + 2] = antennas[i].gpio_mask_lsb;
        tlv.value[off + 3] = antennas[i].gpio_mask_msb;
        tlv.value[off + 4] = antennas[i].gpio_state_lsb;
        tlv.value[off + 5] = antennas[i].gpio_mask_msb;
    }

    return uci_cmd_set_config(&tlv, 1);
}

uci_status_t uci_sr250_set_rx_antenna_pairs(const uci_rx_antenna_pair_t *pairs, uint8_t count)
{
    if (pairs == NULL || count == 0) return UCI_STATUS_INVALID_PARAM;

    uci_tlv_t tlv;
    tlv.tag = ((uint16_t)NXP_CFG_EXT_TAG_PREFIX << 8) | NXP_CFG_ANTENNAS_RX_PAIR_DEFINE;
    tlv.len = (count * 6) + 1;

    tlv.value[0] = count;
    for (uint8_t i = 0; i < count && i * 6 < UCI_MAX_TLV_VALUE_LEN; i++) {
        uint8_t off = (i * 6) + 1;
        tlv.value[off + 0] = pairs[i].pair_id;
        tlv.value[off + 1] = pairs[i].antenna_rxc;
        tlv.value[off + 2] = pairs[i].antenna_rxb;
        tlv.value[off + 3] = pairs[i].antenna_rxa;
        tlv.value[off + 4] = pairs[i].rfu_lsb;
        tlv.value[off + 5] = pairs[i].rfu_msb;
    }

    return uci_cmd_set_config(&tlv, 1);
}

/* ========================================================================== */
/*  Vendor App Configuration                                                  */
/* ========================================================================== */

uci_status_t uci_sr250_set_vendor_app_config(uint32_t session_handle,
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

    return uci_core_send_command(UCI_GID_VENDOR_F, UCI_OID_NXP_SET_VENDOR_APP_CONFIG,
                                 payload, offset, rsp, &rsp_len,
                                 UCI_CMD_RESPONSE_TIMEOUT_MS);
}

uci_status_t uci_sr250_get_vendor_app_config(uint32_t session_handle,
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

    uci_status_t st = uci_core_send_command(UCI_GID_VENDOR_F,
                                             UCI_OID_NXP_GET_VENDOR_APP_CONFIG,
                                             payload, offset, rsp, &rsp_len,
                                             UCI_CMD_RESPONSE_TIMEOUT_MS);
    if (st != UCI_STATUS_OK) return st;

    /* RSP: [status:1] [num_params:1] [TLV data...] */
    if (rsp_len < 2) return UCI_STATUS_FAILED;

    *out_num_tlvs = uci_tlv_deserialize(&rsp[2], rsp_len - 2, out_tlvs, *out_num_tlvs);
    return UCI_STATUS_OK;
}

/* ========================================================================== */
/*  Calibration                                                               */
/* ========================================================================== */

uci_status_t uci_sr250_do_calibration(uint8_t channel)
{
    /* DO_CALIBRATION_CMD (GID 0x0F, OID 0x20):
     *   [channel:1] [calib_param_tag:2 LE] = [channel, 0x00, 0x00] for chip calib */
    uint8_t payload[1];
    payload[0] = channel;
//    put_le16(&payload[1], NXP_CALIB_CHIP_CALIBRATION);

    uint8_t rsp[64];
    uint16_t rsp_len = sizeof(rsp);

    /* Calibration may take several seconds */
    return uci_core_send_command(UCI_GID_VENDOR_F, UCI_OID_NXP_DO_CALIBRATION,
                                 payload, 1, rsp, &rsp_len,
                                 10000);  /* 10 second timeout for calibration */
}

uci_status_t uci_sr250_set_calibration(uint8_t channel, uint16_t tag,
                                        const uint8_t *value, uint8_t len)
{
    if (value == NULL || len == 0) return UCI_STATUS_INVALID_PARAM;

    /* SET_CALIBRATION_CMD (GID 0x0F, OID 0x21):
     *   [channel:1] [param_tag:2 LE] [length:1] [value:N] */
    uint8_t payload[UCI_MAX_PAYLOAD_SIZE];
    uint16_t offset = 0;

    payload[offset++] = channel;
    put_le16(&payload[offset], tag);
    offset += 2;
    payload[offset++] = len;
    memcpy(&payload[offset], value, len);
    offset += len;

    uint8_t rsp[16];
    uint16_t rsp_len = sizeof(rsp);

    return uci_core_send_command(UCI_GID_VENDOR_F, UCI_OID_NXP_SET_CALIBRATION,
                                 payload, offset, rsp, &rsp_len,
                                 UCI_CMD_RESPONSE_TIMEOUT_MS);
}

uci_status_t uci_sr250_get_calibration(uint8_t channel, uint16_t tag,
                                        uint8_t *value, uint8_t *len)
{
    if (value == NULL || len == NULL) return UCI_STATUS_INVALID_PARAM;

    /* GET_CALIBRATION_CMD (GID 0x0F, OID 0x22):
     *   [channel:1] [param_tag:2 LE] */
    uint8_t payload[3];
    payload[0] = channel;
    put_le16(&payload[1], tag);

    uint8_t rsp[256];
    uint16_t rsp_len = sizeof(rsp);

    uci_status_t st = uci_core_send_command(UCI_GID_VENDOR_F, UCI_OID_NXP_GET_CALIBRATION,
                                             payload, 3, rsp, &rsp_len,
                                             UCI_CMD_RESPONSE_TIMEOUT_MS);
    if (st != UCI_STATUS_OK) return st;

    /* RSP: [status:1] [channel:1] [param_tag:2] [length:1] [value:N] */
    if (rsp_len < 5) return UCI_STATUS_FAILED;

    uint8_t val_len = rsp[4];
    if (val_len > *len) val_len = *len;
    if (rsp_len >= 5 + val_len) {
        memcpy(value, &rsp[5], val_len);
    }
    *len = val_len;

    return UCI_STATUS_OK;
}

uci_status_t uci_sr250_query_temperature(int8_t *temperature_c)
{
    if (temperature_c == NULL) return UCI_STATUS_INVALID_PARAM;

    /* QUERY_TEMPERATURE_CMD (GID 0x0E, OID 0x0B): no payload */
    uint8_t rsp[8];
    uint16_t rsp_len = sizeof(rsp);

    uci_status_t st = uci_core_send_command(UCI_GID_VENDOR_E, UCI_OID_NXP_QUERY_TEMPERATURE,
                                             NULL, 0, rsp, &rsp_len,
                                             UCI_CMD_RESPONSE_TIMEOUT_MS);
    if (st != UCI_STATUS_OK) return st;

    /* RSP: [status:1] [temperature:1 signed] */
    if (rsp_len >= 2) {
        *temperature_c = (int8_t)rsp[1];
    }
    return UCI_STATUS_OK;
}

/* ========================================================================== */
/*  Full Initialization Sequence                                              */
/* ========================================================================== */

/* Wait for a specific device state, polling uci_core_process() */
static uci_status_t wait_for_device_state(uint8_t expected_state, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while (uci_core_get_device_state() != expected_state) {
        uci_core_process();
        if ((HAL_GetTick() - start_tick) >= timeout_ms) {
            return UCI_STATUS_TIMEOUT;
        }
    }
    return UCI_STATUS_OK;
}

uci_status_t uci_sr250_full_init(const uci_sr250_init_config_t *config)
{
    if (config == NULL) return UCI_STATUS_INVALID_PARAM;

    uci_status_t st;

    /* ── Step 1: Hardware reset ── */
    uci_transport_hw_reset();

    /* ── Step 2: Wait for CORE_DEVICE_STATUS_NTF with READY ── */
    st = wait_for_device_state(UCI_DEVICE_STATUS_READY, 3000);
    if (st != UCI_STATUS_OK) return st;

    printf("Device out of HW reset and ready.\n\r");

    st = uci_sr250_device_reset();
    if (st != UCI_STATUS_OK) return st;

    st = wait_for_device_state(UCI_DEVICE_STATUS_READY, 3000);
    if (st != UCI_STATUS_OK) return st;

    printf("Device out of SW reset and ready.\n\r");

    /* ── Step 3: NXP CORE_DEVICE_INIT_CMD ── */
    st = uci_sr250_device_init(0x00, 0x00);
    if (st != UCI_STATUS_OK) return st;

    printf("Core device init command issued.\n\r");

    /* ── Step 4: Wait for CORE_DEVICE_STATUS_NTF with READY again ── */
    st = wait_for_device_state(UCI_DEVICE_STATUS_READY, 3000);
    if (st != UCI_STATUS_OK) return st;

    printf("Device out of core init and ready.\n\r");


//    st = uci_sr250_get_device_info();
//    if (st != UCI_STATUS_OK) return st;
    HAL_Delay(100);
    /* ── Step 5: Set RX antenna definitions ── */
    if (config->rx_antennas != NULL && config->num_rx_antennas > 0) {
        st = uci_sr250_set_rx_antenna_defs(config->rx_antennas, config->num_rx_antennas);
        if (st != UCI_STATUS_OK) return st;
    }

    printf("Rx antenna configuration successfully written.\n\r");

    HAL_Delay(100);
    /* ── Step 6: Set TX antenna definitions ── */
    if (config->tx_antennas != NULL && config->num_tx_antennas > 0) {
        st = uci_sr250_set_tx_antenna_defs(config->tx_antennas, config->num_tx_antennas);
        if (st != UCI_STATUS_OK) return st;
    }

    printf("Tx antenna configuration successfully written.\n\r");

    HAL_Delay(100);

    /* ── Step 7: Set RX antenna pairs ── */
    if (config->rx_pairs != NULL && config->num_rx_pairs > 0) {
        st = uci_sr250_set_rx_antenna_pairs(config->rx_pairs, config->num_rx_pairs);
        if (st != UCI_STATUS_OK) return st;
    }

    printf("Rx antenna pairs configuration successfully written.\n\r");

//    /* ── Step 8: Chip calibration (optional) ── */
//    if (config->run_chip_calibration) {
//        st = uci_sr250_do_calibration(config->channel);
//        if (st != UCI_STATUS_OK && st != UCI_STATUS_CALIBRATION_IN_PROGRESS) {
//            return st;
//        }
//    }
    HAL_Delay(100);
    /* ── Step 9: Get device info (verify FW version) ── */
    uci_device_info_t dev_info;
    st = uci_cmd_get_device_info(&dev_info);
    if (st != UCI_STATUS_OK) return st;

    return UCI_STATUS_OK;
}

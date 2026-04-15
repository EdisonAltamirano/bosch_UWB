/*
 * uci_radar.c
 *
 * Radar mode implementation (PRIMARY use case).
 *
 * Configuration sequence follows the reference demos (demo_radar.c, demo_ocpd.c):
 *   1. Set standard APP configs (CHANNEL_NUMBER, PREAMBLE_CODE_INDEX)
 *   2. Set vendor APP configs (ANT_RX, ANT_TX, RADAR_MODE, RFRI, etc.)
 *   3. Start session
 *
 * RADAR_RX_NTF (GID 0x0A, OID 0x0A) is parsed for all 4 data types.
 *
 * CIR sample format:
 *   TAP[0]-TAP[7]: 32 bytes of metadata (parsed into uci_radar_cir_metadata_t)
 *   TAP[8+]:        Actual CIR complex samples (3 bytes per TAP for 16-bit res)
 */

#include "uci/uci_radar.h"
#include "uci/uci_commands.h"
#include "uci/uci_core.h"
#include "uci/uci_defs.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* Forward declaration for vendor app config (implemented in uci_sr250.c) */
extern uci_status_t uci_sr250_set_vendor_app_config(uint32_t session_handle,
                                                      const uci_tlv_t *tlvs,
                                                      uint8_t num_tlvs);

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

static inline uint32_t get_le32(const uint8_t *buf)
{
    return (uint32_t)buf[0] |
           ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) |
           ((uint32_t)buf[3] << 24);
}

/* ========================================================================== */
/*  Radar Session Lifecycle                                                   */
/* ========================================================================== */

uci_status_t uci_radar_session_init(uint32_t session_id, uint32_t *out_session_handle)
{
    return uci_cmd_session_init(session_id, UCI_SESSION_TYPE_RADAR, out_session_handle);
}

uci_status_t uci_radar_start(uint32_t session_handle)
{
    return uci_cmd_session_start(session_handle);
}

uci_status_t uci_radar_stop(uint32_t session_handle)
{
    return uci_cmd_session_stop(session_handle);
}

uci_status_t uci_radar_deinit(uint32_t session_handle)
{
    return uci_cmd_session_deinit(session_handle);
}

/* ========================================================================== */
/*  Radar Session Configuration                                               */
/* ========================================================================== */

uci_status_t uci_radar_configure(uint32_t session_handle, const uci_radar_params_t *params)
{
    if (params == NULL) return UCI_STATUS_INVALID_PARAM;

    uci_status_t st;

    /* ── Validate AoA constraints ── */
    if (params->presence_det.mode & 0x02) {
        /* AoA requested: need at least two RX antennas */
        uint8_t rx_count = 0;
        if (params->ant_rx_rxc_id != 0) rx_count++;
        if (params->ant_rx_rxb_id != 0) rx_count++;
        if (params->ant_rx_rxa_id != 0) rx_count++;
        if (rx_count < 2) {
            return UCI_STATUS_INVALID_PARAM;
        }
    }

    /* ── Validate OCPD constraint: RFRI must be 50ms when presence detection enabled ── */
    if ((params->presence_det.mode & 0x01) && params->use_rfri) {
        if (params->rfri.ranging_interval_ms != 50) {
            return UCI_STATUS_INVALID_PARAM;
        }
    }

    /* ── Step 1: Set standard APP configuration ── */
    {
        uci_tlv_t app_tlvs[3];
        uint8_t n = 0;

        uci_tlv_build_u8(&app_tlvs[n++], UCI_PARAM_CHANNEL_NUMBER,
                          params->channel_number);
        uci_tlv_build_u8(&app_tlvs[n++], UCI_PARAM_PREAMBLE_CODE_INDEX,
                          params->preamble_code_index);
        /* MAX_NUMBER_OF_MEASUREMENTS (tag 0x32) — if non-zero */
        if (params->max_measurements != 0) {
            uci_tlv_build_u16(&app_tlvs[n++], UCI_PARAM_MAX_NUM_OF_MEASUREMENTS,
                               params->max_measurements);
        }

        st = uci_cmd_session_set_app_config(session_handle, app_tlvs, n);
        if (st != UCI_STATUS_OK) return st;
    }

    /* ── Step 2: Set vendor APP configuration ── */
    {
        uci_tlv_t vtlvs[16];
        uint8_t n = 0;

        /* ANTENNAS_CONFIGURATION_RX (tag 0x03): [mode, length, rxc, rxb, rxa] */
        {
            uint8_t ant_rx[5];
            ant_rx[0] = NXP_ANT_RX_MODE_RADAR;  /* mode = 0x02 (radar) */
            ant_rx[1] = 0x03;                     /* length of antenna IDs */
            ant_rx[2] = params->ant_rx_rxc_id;
            ant_rx[3] = params->ant_rx_rxb_id;
            ant_rx[4] = params->ant_rx_rxa_id;
            uci_tlv_build_array(&vtlvs[n++], NXP_PARAM_ANTENNAS_CONFIG_RX, ant_rx, 5);
        }

        /* ANTENNAS_CONFIGURATION_TX (tag 0x02): [length, tx_id] */
        {
            uint8_t ant_tx[2];
            ant_tx[0] = 0x01;                     /* length = 1 antenna */
            ant_tx[1] = params->ant_tx_id;
            uci_tlv_build_array(&vtlvs[n++], NXP_PARAM_ANTENNAS_CONFIG_TX, ant_tx, 2);
        }

        /* RADAR_MODE (tag 0xA0) — mandatory */
        uci_tlv_build_u8(&vtlvs[n++], NXP_RADAR_PARAM_MODE, (uint8_t)params->mode);

        /* RADAR_SINGLE_FRAME_NTF (tag 0xA5) */
        uci_tlv_build_u8(&vtlvs[n++], NXP_RADAR_PARAM_SINGLE_FRAME_NTF,
                          params->single_frame_ntf);

        /* RADAR_RFRI (tag 0xA9): [ranging_interval:4 LE] [slot_duration:2 LE] [slots_per_rr:1] */
        if (params->use_rfri) {
            uint8_t rfri_buf[7];
            put_le32(&rfri_buf[0], params->rfri.ranging_interval_ms);
            put_le16(&rfri_buf[4], params->rfri.slot_duration_rstu);
            rfri_buf[6] = params->rfri.slots_per_rr;
            uci_tlv_build_array(&vtlvs[n++], NXP_RADAR_PARAM_RFRI, rfri_buf, 7);
        }

        /* RADAR_CIR_NUM_SAMPLES (tag 0xA7) — if non-default */
        if (params->cir_num_samples != 0) {
            uci_tlv_build_u8(&vtlvs[n++], NXP_RADAR_PARAM_CIR_NUM_SAMPLES,
                              params->cir_num_samples);
        }

        /* RADAR_RX_GAIN (tag 0xA4): [agc_mode, gain_rxa, gain_rxb, gain_rxc] */
        if (params->rx_gain.agc_mode != 0 ||
            params->rx_gain.gain_rxa != 0 ||
            params->rx_gain.gain_rxb != 0 ||
            params->rx_gain.gain_rxc != 0) {
            uint8_t gain_buf[4];
            gain_buf[0] = params->rx_gain.agc_mode;
            gain_buf[1] = params->rx_gain.gain_rxa;
            gain_buf[2] = params->rx_gain.gain_rxb;
            gain_buf[3] = params->rx_gain.gain_rxc;
            uci_tlv_build_array(&vtlvs[n++], NXP_RADAR_PARAM_RX_GAIN, gain_buf, 4);
        }

        /* RADAR_CIR_START_OFFSET (tag 0xA8): [rxc:2 LE] [rxb:2 LE] [rxa:2 LE] */
        if (params->cir_start_offset.rxc_start_index != 0 ||
            params->cir_start_offset.rxb_start_index != 0 ||
            params->cir_start_offset.rxa_start_index != 0) {
            uint8_t ofs_buf[6];
            put_le16(&ofs_buf[0], params->cir_start_offset.rxc_start_index);
            put_le16(&ofs_buf[2], params->cir_start_offset.rxb_start_index);
            put_le16(&ofs_buf[4], params->cir_start_offset.rxa_start_index);
            uci_tlv_build_array(&vtlvs[n++], NXP_RADAR_PARAM_CIR_START_OFFSET, ofs_buf, 6);
        }

        /* RADAR_PERFORMANCE (tag 0xAD) — if non-default */
        if (params->performance != 0) {
            uci_tlv_build_u8(&vtlvs[n++], NXP_RADAR_PARAM_PERFORMANCE,
                              params->performance);
        }

        /* RADAR_DRIFT_COMPENSATION (tag 0xB2) — always set */
        uci_tlv_build_u16(&vtlvs[n++], NXP_RADAR_PARAM_DRIFT_COMPENSATION,
                           params->drift_compensation);

        /* RADAR_PRESENCE_DET_CFG (tag 0xAA): 12 bytes if enabled */
        if (params->presence_det.mode != 0) {
            uint8_t pres_buf[12];
            pres_buf[0] = params->presence_det.mode;
            pres_buf[1] = params->presence_det.periodic_report;
            pres_buf[2] = params->presence_det.sensitivity_q4_4;
            pres_buf[3] = params->presence_det.gpio_notify;
            put_le16(&pres_buf[4], params->presence_det.distance_min_cm);
            put_le16(&pres_buf[6], params->presence_det.distance_max_cm);
            put_le16(&pres_buf[8], params->presence_det.hold_delay_ms);
            pres_buf[10] = (uint8_t)params->presence_det.angle_min_deg;
            pres_buf[11] = (uint8_t)params->presence_det.angle_max_deg;
            uci_tlv_build_array(&vtlvs[n++], NXP_RADAR_PARAM_PRESENCE_DET_CFG, pres_buf, 12);
        }

        st = uci_sr250_set_vendor_app_config(session_handle, vtlvs, n);
        if (st != UCI_STATUS_OK) return st;
    }

    return UCI_STATUS_OK;
}

/* ========================================================================== */
/*  Convenience — Presence Detection with AoA                                 */
/* ========================================================================== */

uci_status_t uci_radar_configure_presence_aoa(
    uint32_t session_handle,
    uint8_t  channel,
    uint8_t  ant_tx_id,
    uint8_t  ant_rxb_id,
    uint8_t  ant_rxc_id,
    uint16_t distance_min_cm,
    uint16_t distance_max_cm,
    int8_t   angle_min_deg,
    int8_t   angle_max_deg)
{
    if (ant_rxb_id == 0 || ant_rxc_id == 0 || ant_tx_id == 0) {
        return UCI_STATUS_INVALID_PARAM;
    }

    uci_radar_params_t params;
    memset(&params, 0, sizeof(params));

    params.mode                = UCI_RADAR_MODE_MEDIUM_DISTANCE;  /* Required for OCPD */
    params.channel_number      = channel;
    params.preamble_code_index = 26;
    params.cir_num_samples     = 128;
    params.single_frame_ntf    = 1;
    params.performance         = 0x03;   /* DC freeze + BW increase */
    params.drift_compensation  = 0x0CCD; /* Q1.15 of 0.1 */

    /* 50ms RFRI required for OCPD */
    params.use_rfri = true;
    params.rfri.ranging_interval_ms = 50;
    params.rfri.slot_duration_rstu  = 12000;
    params.rfri.slots_per_rr        = 1;

    /* Two RX antennas for AoA */
    params.ant_rx_rxc_id = ant_rxc_id;
    params.ant_rx_rxb_id = ant_rxb_id;
    params.ant_rx_rxa_id = 0;
    params.ant_tx_id     = ant_tx_id;

    /* Presence detection with AoA */
    params.presence_det.mode             = 0x03;  /* Bit0: enable, Bit1: distance+AoA */
    params.presence_det.periodic_report  = 0x04;  /* 400ms reporting */
    params.presence_det.sensitivity_q4_4 = 60;    /* Q4.4 of 3.75 */
    params.presence_det.gpio_notify      = 0x00;
    params.presence_det.distance_min_cm  = distance_min_cm;
    params.presence_det.distance_max_cm  = distance_max_cm;
    params.presence_det.hold_delay_ms    = 1600;
    params.presence_det.angle_min_deg    = angle_min_deg;
    params.presence_det.angle_max_deg    = angle_max_deg;

    params.max_measurements = 0; /* Unlimited */

    return uci_radar_configure(session_handle, &params);
}

/* ========================================================================== */
/*  RADAR_RX_NTF Parsing                                                      */
/* ========================================================================== */

/*
 * RADAR_RX_NTF payload layout (GID 0x0A, OID 0x0A):
 *   [session_handle:4] [status:1] [radar_data_type:1] [data...]
 */

uci_radar_data_type_t uci_radar_get_ntf_type(const uint8_t *payload, uint16_t len)
{
    if (payload == NULL || len < 6) {
        return (uci_radar_data_type_t)0xFF;
    }
    /* radar_data_type is at offset 5 (after session_handle[4] + status[1]) */
    return (uci_radar_data_type_t)payload[5];
}

/*
 * Parse CIR samples notification (data type 0x00).
 *
 * After the 6-byte common header:
 *   [num_cir_samples:2 LE] [taps_per_sample:1] [cir_data...]
 *
 * cir_data contains for each CIR sample:
 *   TAP[0]-TAP[7]: 8 metadata words (4 bytes each = 32 bytes)
 *   TAP[8]-TAP[N]: Actual CIR complex data (3 bytes per TAP for 16-bit resolution)
 *
 * TAP metadata (32 bytes):
 *   TAP[0]: cir_counter[15:0] (bytes 0-1), rx_timestamp[15:0] (bytes 2-3)
 *   TAP[1]: rx_timestamp[31:16] (bytes 0-1), tx_timestamp[15:0] (bytes 2-3)
 *   TAP[2]: tx_timestamp[31:16] (bytes 0-1), flags (bytes 2-3)
 *   TAP[3]: gain info (bytes 0-3)
 *   TAP[4]: antenna IDs (bytes 0-3)
 *   TAP[5]: mode/flags (bytes 0-3)
 *   TAP[6]: cir_start_offset (bytes 0-1), zero_dist_offset (bytes 2-3)
 *   TAP[7]: reserved
 */
uci_status_t uci_radar_parse_cir_ntf(
    const uint8_t *payload, uint16_t len,
    uci_radar_cir_metadata_t *metadata,
    const uint8_t **cir_data_out,
    uint16_t *num_cir_taps)
{
    if (payload == NULL || metadata == NULL || cir_data_out == NULL || num_cir_taps == NULL) {
        return UCI_STATUS_INVALID_PARAM;
    }
    if (len < 9) return UCI_STATUS_INVALID_MSG_SIZE;

    /* Common header: session_handle[4] + status[1] + radar_data_type[1] */
    uint16_t off = 6;

    /* num_cir_samples[2 LE] + taps_per_sample[1] */
    if (off + 3 > len) return UCI_STATUS_INVALID_MSG_SIZE;
    uint16_t num_samples = get_le16(&payload[off]); off += 2;
    uint8_t taps_per_sample = payload[off++];
    /* Advancing one counter step for RFU byte */
    off = off + 1;

    /* CIR data starts here — first 32 bytes are TAP[0]-TAP[7] metadata */
    const uint8_t *cir_start = &payload[off];
    uint16_t cir_data_len = len - off;

    /* Parse metadata from TAP[0]-TAP[7] (32 bytes) */
    if (cir_data_len < 32) {
        /* Insufficient data for metadata */
        memset(metadata, 0, sizeof(*metadata));
        *cir_data_out = NULL;
        *num_cir_taps = 0;
        return UCI_STATUS_INVALID_MSG_SIZE;
    }

    /* TAP[0] bytes 0-3: cir_counter[15:0], rx_timestamp[15:0] */
    metadata->cir_counter   = get_le32(&cir_start[0]);
    metadata->rx_timestamp       = get_le32(&cir_start[4]);
    metadata->tx_timestamp       = get_le32(&cir_start[12]);
    metadata->tap_resolution_bits = cir_start[11] & 0x0C;
    metadata->tap_resolution_cm   = cir_start[11] & 0x03;
    metadata->tx_gain_index = cir_start[9];
    metadata->rx_gain_index = cir_start[8];
    metadata->tx_antenna_id = cir_start[19];
    metadata->rx_antenna_id = cir_start[18];
    metadata->rx_path       = cir_start[16];
    metadata->radar_mode       = cir_start[17] & 0x0F;
    metadata->inter_pulse_distance_in_ns = cir_start[20];
    metadata->correlation_bit_shift = cir_start[21];
    metadata->log_2_delta_L = cir_start[22] & 0x3F;
    /* TAP[3] bytes 0-3: gain indices */
    metadata->tx_pulse_shape_id = cir_start[10];
    metadata->dc_freeze_enabled = ((cir_start[17] & 0x20) >> 5) ? true : false;
    metadata->single_pin        = ((cir_start[17] & 0x10) >> 4) ? true : false;
    metadata->nsync_by_8 = (get_le16(&cir_start[22]) & 0xFFC0) >> 6;
    metadata->peak_real_cir = get_le16(&cir_start[24]);
    metadata->num_pulses_used = get_le16(&cir_start[26]);
    metadata->cir_start_offset = get_le16(&cir_start[30]);
    metadata->cir_start_offset_fractional = get_le16(&cir_start[28]);

    /* Actual CIR TAP data starts after 32 bytes of metadata (TAP[8]+) */
    uint16_t metadata_bytes = 32;
    if (cir_data_len > metadata_bytes) {
        *cir_data_out = &cir_start[metadata_bytes];
        /* Each TAP is 3 bytes for 16-bit complex resolution */
        uint16_t actual_cir_bytes = cir_data_len - metadata_bytes;
        uint8_t bytes_per_tap = 3; /* Default 16-bit complex */
//        if (tap_res_bits == 0x01) bytes_per_tap = 4;  /* 24-bit */
//        if (tap_res_bits == 0x02) bytes_per_tap = 4;  /* 32-bit */
        *num_cir_taps = (bytes_per_tap > 0) ? (actual_cir_bytes / bytes_per_tap) : 0;
    } else {
        *cir_data_out = NULL;
        *num_cir_taps = 0;
    }

    (void)num_samples;
    (void)taps_per_sample;

    return UCI_STATUS_OK;
}

/*
 * Parse presence detection notification (data type 0x01).
 *
 * After the 6-byte common header:
 *   [presence_detected:1] [presence_mode:1] [num_detections:1]
 *   For each detection:
 *     [distance_cm:2 LE] [angle_deg:1 signed] [snr_trigger:4 float] [snr_valid:1]
 */
uci_status_t uci_radar_parse_presence_ntf(
    const uint8_t *payload, uint16_t len,
    uci_radar_presence_ntf_t *result)
{
    if (payload == NULL || result == NULL) return UCI_STATUS_INVALID_PARAM;
    if (len < 9) return UCI_STATUS_INVALID_MSG_SIZE;

    memset(result, 0, sizeof(*result));

    /* Common header */
    result->session_handle = get_le32(&payload[0]);
    result->status         = payload[4];

    uint16_t off = 6;

    result->presence_detected = (payload[off++] != 0);
    result->presence_mode     = payload[off++];
    result->num_detections    = payload[off++];

    if (result->num_detections > UCI_RADAR_MAX_DETECTIONS) {
        result->num_detections = UCI_RADAR_MAX_DETECTIONS;
    }

    for (uint8_t i = 0; i < result->num_detections; i++) {
        if (off + 3 > len) break;

        result->targets[i].distance_cm = get_le16(&payload[off]); off += 2;
        result->targets[i].angle_deg   = (int8_t)payload[off++];

        /* SNR trigger value (IEEE 754 float, 4 bytes LE) */
        if (off + 4 <= len) {
            float snr_val;
            memcpy(&snr_val, &payload[off], 4);
            result->targets[i].snr_trigger = snr_val;
            off += 4;

            if (off < len) {
                result->targets[i].snr_valid = (payload[off++] != 0);
            }
        }
    }

    return UCI_STATUS_OK;
}

/*
 * Parse antenna isolation report (data type 0x20).
 *
 * After the 6-byte common header:
 *   [tx_antenna_id:1] [rx_antenna_id:1] [isolation_db:2 LE]
 */
uci_status_t uci_radar_parse_ant_isolation_ntf(
    const uint8_t *payload, uint16_t len,
    uci_radar_ant_isolation_ntf_t *result)
{
    if (payload == NULL || result == NULL) return UCI_STATUS_INVALID_PARAM;
    if (len < 10) return UCI_STATUS_INVALID_MSG_SIZE;

    memset(result, 0, sizeof(*result));

    result->session_handle = get_le32(&payload[0]);
    result->status         = payload[4];
    /* payload[5] = radar_data_type (0x20) */
    result->tx_antenna_id  = payload[6];
    result->rx_antenna_id  = payload[7];
    result->isolation_db   = get_le16(&payload[8]);

    return UCI_STATUS_OK;
}

/* ========================================================================== */
/*  CIR TAP Extraction                                                        */
/* ========================================================================== */

void uci_radar_get_cir_tap(const uint8_t *cir_data, uint16_t tap_index,
                            int16_t *real, int16_t *imag)
{
    if (cir_data == NULL || real == NULL || imag == NULL) return;

    /* Default 16-bit complex resolution: 3 bytes per TAP
     * Byte layout: [real_lo] [real_hi] [imag_lo]
     * Real: signed 16-bit (bytes 0-1), Imag: signed 8-bit sign-extended (byte 2) */
    uint16_t offset = tap_index * 3;

    /* 16-bit real value (little-endian) */
    *real = (int16_t)((uint16_t)cir_data[offset] | ((uint16_t)cir_data[offset + 1] << 8));

    /* 8-bit imaginary value sign-extended to 16-bit */
    *imag = (int16_t)(int8_t)cir_data[offset + 2];
}

/* ========================================================================== */
/*  Runtime Parameter Updates                                                 */
/* ========================================================================== */

uci_status_t uci_radar_set_rx_gain(uint32_t session_handle,
                                    const uci_radar_rx_gain_t *gain)
{
    if (gain == NULL) return UCI_STATUS_INVALID_PARAM;

    uci_tlv_t tlv;
    uint8_t buf[4] = {
        gain->agc_mode,
        gain->gain_rxa,
        gain->gain_rxb,
        gain->gain_rxc
    };
    uci_tlv_build_array(&tlv, NXP_RADAR_PARAM_RX_GAIN, buf, 4);
    return uci_sr250_set_vendor_app_config(session_handle, &tlv, 1);
}

uci_status_t uci_radar_set_rfri(uint32_t session_handle,
                                 const uci_radar_rfri_t *rfri)
{
    if (rfri == NULL) return UCI_STATUS_INVALID_PARAM;

    uci_tlv_t tlv;
    uint8_t buf[7];
    put_le32(&buf[0], rfri->ranging_interval_ms);
    put_le16(&buf[4], rfri->slot_duration_rstu);
    buf[6] = rfri->slots_per_rr;
    uci_tlv_build_array(&tlv, NXP_RADAR_PARAM_RFRI, buf, 7);
    return uci_sr250_set_vendor_app_config(session_handle, &tlv, 1);
}

uci_status_t uci_radar_set_cir_start_offset(uint32_t session_handle,
                                              const uci_radar_cir_start_offset_t *offset)
{
    if (offset == NULL) return UCI_STATUS_INVALID_PARAM;

    uci_tlv_t tlv;
    uint8_t buf[6];
    put_le16(&buf[0], offset->rxc_start_index);
    put_le16(&buf[2], offset->rxb_start_index);
    put_le16(&buf[4], offset->rxa_start_index);
    uci_tlv_build_array(&tlv, NXP_RADAR_PARAM_CIR_START_OFFSET, buf, 6);
    return uci_sr250_set_vendor_app_config(session_handle, &tlv, 1);
}

uci_status_t uci_radar_update_presence_cfg(uint32_t session_handle,
                                            const uci_radar_presence_cfg_t *cfg)
{
    if (cfg == NULL) return UCI_STATUS_INVALID_PARAM;

    uci_tlv_t tlv;
    uint8_t buf[12];
    buf[0] = cfg->mode;
    buf[1] = cfg->periodic_report;
    buf[2] = cfg->sensitivity_q4_4;
    buf[3] = cfg->gpio_notify;
    put_le16(&buf[4], cfg->distance_min_cm);
    put_le16(&buf[6], cfg->distance_max_cm);
    put_le16(&buf[8], cfg->hold_delay_ms);
    buf[10] = (uint8_t)cfg->angle_min_deg;
    buf[11] = (uint8_t)cfg->angle_max_deg;
    uci_tlv_build_array(&tlv, NXP_RADAR_PARAM_PRESENCE_DET_CFG, buf, 12);
    return uci_sr250_set_vendor_app_config(session_handle, &tlv, 1);
}

/* ========================================================================== */
/*  Antenna Isolation Test                                                    */
/* ========================================================================== */

/* Internal state for capturing the ant isolation NTF result */
static volatile bool s_ant_iso_received;
static uci_radar_ant_isolation_ntf_t s_ant_iso_result;

static void ant_iso_ntf_handler(uint8_t gid, uint8_t oid,
                                 const uint8_t *payload, uint16_t len)
{
    if (gid == UCI_GID_VENDOR_NXP_PROP && oid == UCI_OID_RADAR_RX_NTF) {
        if (len >= 6 && payload[5] == UCI_RADAR_DATA_ANT_ISOLATION) {
            uci_radar_parse_ant_isolation_ntf(payload, len, &s_ant_iso_result);
            s_ant_iso_received = true;
        }
    }
}

uci_status_t uci_radar_run_ant_isolation_test(
    uint8_t channel, uint8_t ant_tx_id, uint8_t ant_rx_id,
    uci_radar_ant_isolation_ntf_t *result)
{
    if (result == NULL) return UCI_STATUS_INVALID_PARAM;

    uci_status_t st;
    uint32_t session_handle;

    /* Create a temporary radar session */
    st = uci_radar_session_init(0xAA550001, &session_handle);
    if (st != UCI_STATUS_OK) return st;

    /* Configure for antenna isolation mode */
    uci_radar_params_t params;
    memset(&params, 0, sizeof(params));
    params.mode                = UCI_RADAR_MODE_ANT_ISOLATION;
    params.channel_number      = channel;
    params.preamble_code_index = 26;
    params.cir_num_samples     = 0;
    params.single_frame_ntf    = 1;
    params.performance         = 0x03;
    params.drift_compensation  = 0x01;  /* Enabled */
    params.ant_rx_rxc_id       = ant_rx_id;
    params.ant_rx_rxb_id       = 0;
    params.ant_rx_rxa_id       = 0;
    params.ant_tx_id           = ant_tx_id;
    params.max_measurements    = 1;

    st = uci_radar_configure(session_handle, &params);
    if (st != UCI_STATUS_OK) {
        uci_radar_deinit(session_handle);
        return st;
    }

    /* Install temporary NTF handler */
    s_ant_iso_received = false;
    uci_ntf_callback_t prev_cb = NULL;
    /* Note: we save/restore the callback via a simple approach — register ours */
    uci_core_register_ntf_callback(ant_iso_ntf_handler);

    /* Start the session */
    st = uci_radar_start(session_handle);
    if (st != UCI_STATUS_OK) {
        uci_radar_deinit(session_handle);
        return st;
    }

    /* Poll for result (timeout 5 seconds) */
    uint32_t start_tick = HAL_GetTick();
    while (!s_ant_iso_received) {
        uci_core_process();
        if ((HAL_GetTick() - start_tick) >= 5000) {
            uci_radar_stop(session_handle);
            uci_radar_deinit(session_handle);
            return UCI_STATUS_TIMEOUT;
        }
    }

    /* Copy result */
    memcpy(result, &s_ant_iso_result, sizeof(*result));

    /* Cleanup */
    uci_radar_stop(session_handle);
    uci_radar_deinit(session_handle);

    /* Restore previous callback (caller should re-register theirs) */
    uci_core_register_ntf_callback(prev_cb);

    return UCI_STATUS_OK;
}

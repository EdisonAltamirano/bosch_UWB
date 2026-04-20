/*
 * uci_commands.h
 *
 * High-level UCI command builder API.
 * Constructs command payloads, sends via uci_core, and parses responses.
 *
 * Covers: Core (GID 0x00), Session Config (GID 0x01), Session Control (GID 0x02).
 * Vendor-specific commands are in uci_sr250.h; radar commands in uci_radar.h.
 */

#ifndef UCI_COMMANDS_H
#define UCI_COMMANDS_H

#include <stdint.h>
#include "uci_defs.h"
#include "uci_types.h"

/* ========================================================================== */
/*  TLV Helpers                                                               */
/* ========================================================================== */

/* Build a TLV with a uint8_t value */
void uci_tlv_build_u8(uci_tlv_t *tlv, uint16_t tag, uint8_t value);

/* Build a TLV with a uint16_t value (little-endian) */
void uci_tlv_build_u16(uci_tlv_t *tlv, uint16_t tag, uint16_t value);

/* Build a TLV with a uint32_t value (little-endian) */
void uci_tlv_build_u32(uci_tlv_t *tlv, uint16_t tag, uint32_t value);

/* Build a TLV with a byte array */
void uci_tlv_build_array(uci_tlv_t *tlv, uint16_t tag, const uint8_t *data, uint8_t len);

/* Serialize a TLV array into a byte buffer. Returns bytes written. */
uint16_t uci_tlv_serialize(const uci_tlv_t *tlvs, uint8_t num_tlvs, uint8_t *buf);

/* Deserialize a byte buffer into a TLV array. Returns number of TLVs parsed. */
uint8_t uci_tlv_deserialize(const uint8_t *buf, uint16_t len,
                             uci_tlv_t *tlvs, uint8_t max_tlvs);

/* ========================================================================== */
/*  Core Group (GID 0x00)                                                     */
/* ========================================================================== */

/* Reset the UWBS device */
uci_status_t uci_cmd_device_reset(void);

/* Get device info (UCI/MAC/PHY versions, vendor info) */
uci_status_t uci_cmd_get_device_info(uci_device_info_t *info);

/* Get device capabilities (raw TLV buffer) */
uci_status_t uci_cmd_get_caps_info(uint8_t *caps_tlv_buf, uint16_t *caps_len);

/* Set device configuration (TLV list) */
uci_status_t uci_cmd_set_config(const uci_tlv_t *tlvs, uint8_t num_tlvs);

/* Get device configuration */
uci_status_t uci_cmd_get_config(const uint8_t *param_ids, uint8_t num_params,
                                 uci_tlv_t *out_tlvs, uint8_t *out_num_tlvs);

/* ========================================================================== */
/*  Session Config Group (GID 0x01)                                           */
/* ========================================================================== */

/* Initialize a new session. Returns session handle in *out_session_handle. */
uci_status_t uci_cmd_session_init(uint32_t session_id, uint8_t session_type,
                                   uint32_t *out_session_handle);

/* Deinitialize a session */
uci_status_t uci_cmd_session_deinit(uint32_t session_handle);

/* Set application configuration parameters (TLV list) */
uci_status_t uci_cmd_session_set_app_config(uint32_t session_handle,
                                             const uci_tlv_t *tlvs, uint8_t num_tlvs);

/* Get application configuration parameters */
uci_status_t uci_cmd_session_get_app_config(uint32_t session_handle,
                                             const uint8_t *param_ids, uint8_t num_params,
                                             uci_tlv_t *out_tlvs, uint8_t *out_num_tlvs);

/* Get session count */
uci_status_t uci_cmd_session_get_count(uint8_t *count);

/* Get session state */
uci_status_t uci_cmd_session_get_state(uint32_t session_handle, uint8_t *state);

/* Update controller multicast list */
uci_status_t uci_cmd_session_update_multicast_list(uint32_t session_handle,
                                                    uint8_t action,
                                                    uint8_t num_controlees,
                                                    const uint16_t *controlee_addrs);

/* ========================================================================== */
/*  Session Control Group (GID 0x02)                                          */
/* ========================================================================== */

/* Start a session (ranging or radar) */
uci_status_t uci_cmd_session_start(uint32_t session_handle);

/* Stop a session */
uci_status_t uci_cmd_session_stop(uint32_t session_handle);

/* Get ranging count */
uci_status_t uci_cmd_session_get_ranging_count(uint32_t session_handle, uint32_t *count);

/* ========================================================================== */
/*  Convenience — Ranging (secondary use case)                                */
/* ========================================================================== */

/* Configure ranging parameters using the convenience struct */
uci_status_t uci_cmd_set_ranging_params(uint32_t session_handle,
                                         const uci_ranging_params_t *params);

/* Parse a SESSION_INFO_NTF payload into a structured result */
uci_status_t uci_parse_session_info_ntf(const uint8_t *payload, uint16_t len,
                                         uci_session_info_ntf_t *result);

#endif /* UCI_COMMANDS_H */

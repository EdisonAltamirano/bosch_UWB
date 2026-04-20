/*
 * uci_sr250.h
 *
 * NXP SR250-specific vendor extensions.
 *
 * Provides:
 *   - CORE_DEVICE_INIT_CMD (mandatory after reset)
 *   - Unified antenna configuration (RX defines, TX defines, RX pairs)
 *   - Vendor APP config set/get (GID 0x0F)
 *   - Calibration commands (chip calibration, set/get, temperature)
 *   - Full initialization sequence (reset → init → antenna → calibration)
 */

#ifndef UCI_SR250_H
#define UCI_SR250_H

#include <stdint.h>
#include "uci_defs.h"
#include "uci_types.h"

/* ========================================================================== */
/*  Device Initialization                                                     */
/* ========================================================================== */

/* NXP CORE_DEVICE_INIT_CMD (GID 0x0E, OID 0x00).
 * Must be called immediately after hardware reset, before any other commands.
 * major_version/minor_version: host UCI protocol version (typically 0x00, 0x00). */
uci_status_t uci_sr250_device_init(uint8_t major_version, uint8_t minor_version);

/* ========================================================================== */
/*  Antenna Configuration                                                     */
/* ========================================================================== */

/* Define RX antenna indices (CORE_SET_CONFIG with tag 0xE4/0x60) */
uci_status_t uci_sr250_set_rx_antenna_defs(const uci_rx_antenna_def_t *antennas, uint8_t count);

/* Define TX antenna indices (CORE_SET_CONFIG with tag 0xE4/0x63) */
uci_status_t uci_sr250_set_tx_antenna_defs(const uci_tx_antenna_def_t *antennas, uint8_t count);

/* Define RX antenna pairs for AoA (CORE_SET_CONFIG with tag 0xE4/0x62) */
uci_status_t uci_sr250_set_rx_antenna_pairs(const uci_rx_antenna_pair_t *pairs, uint8_t count);

/* ========================================================================== */
/*  Vendor App Configuration                                                  */
/* ========================================================================== */

/* Set vendor-specific session app config (GID 0x0F, OID 0x00) */
uci_status_t uci_sr250_set_vendor_app_config(uint32_t session_handle,
                                              const uci_tlv_t *tlvs, uint8_t num_tlvs);

/* Get vendor-specific session app config (GID 0x0F, OID 0x03) */
uci_status_t uci_sr250_get_vendor_app_config(uint32_t session_handle,
                                              const uint8_t *param_ids, uint8_t num_params,
                                              uci_tlv_t *out_tlvs, uint8_t *out_num_tlvs);

/* ========================================================================== */
/*  Calibration                                                               */
/* ========================================================================== */

/* Perform chip self-calibration (GID 0x0F, OID 0x20) */
uci_status_t uci_sr250_do_calibration(uint8_t channel);

/* Set calibration parameter (GID 0x0F, OID 0x21) */
uci_status_t uci_sr250_set_calibration(uint8_t channel, uint16_t tag,
                                        const uint8_t *value, uint8_t len);

/* Get calibration parameter (GID 0x0F, OID 0x22) */
uci_status_t uci_sr250_get_calibration(uint8_t channel, uint16_t tag,
                                        uint8_t *value, uint8_t *len);

/* Query device temperature (GID 0x0E, OID 0x0B) */
uci_status_t uci_sr250_query_temperature(int8_t *temperature_c);

/* ========================================================================== */
/*  Full Initialization Sequence                                              */
/* ========================================================================== */

/* Complete SR250 initialization:
 *   1. Hardware reset
 *   2. Wait for CORE_DEVICE_STATUS_NTF (READY)
 *   3. CORE_DEVICE_INIT_CMD
 *   4. Wait for CORE_DEVICE_STATUS_NTF (READY)
 *   5. Set RX antenna definitions
 *   6. Set TX antenna definitions
 *   7. Set RX antenna pairs
 *   8. Chip calibration (if requested)
 *   9. Get device info (verify FW) */
uci_status_t uci_sr250_full_init(const uci_sr250_init_config_t *config);

#endif /* UCI_SR250_H */

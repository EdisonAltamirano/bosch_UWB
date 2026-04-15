/*
 * uci_radar.h
 *
 * Radar mode API (PRIMARY use case).
 *
 * Provides:
 *   - Radar session lifecycle (init, configure, start, stop, deinit)
 *   - RADAR_RX_NTF parsing for all 4 data types:
 *       0x00 = CIR samples (with TAP metadata extraction)
 *       0x01 = Presence detection (with AoA)
 *       0x20 = Antenna isolation
 *       0x21 = LPRF calibration
 *   - Convenience configuration for presence detection with AoA
 *   - Runtime parameter updates (RX gain, RFRI, CIR offset, presence cfg)
 *   - Antenna isolation test helper
 */

#ifndef UCI_RADAR_H
#define UCI_RADAR_H

#include <stdint.h>
#include <stdbool.h>
#include "uci_defs.h"
#include "uci_types.h"

/* ========================================================================== */
/*  Radar Session Lifecycle                                                   */
/* ========================================================================== */

/* Initialize a radar session (session type 0xF0) */
uci_status_t uci_radar_session_init(uint32_t session_id, uint32_t *out_session_handle);

/* Configure a radar session with all parameters at once.
 * Sends standard APP configs + vendor APP configs in the correct order.
 * Validates AoA constraints (two RX antennas required) and
 * OCPD constraints (RFRI=50ms required for presence detection). */
uci_status_t uci_radar_configure(uint32_t session_handle, const uci_radar_params_t *params);

/* Start a radar session */
uci_status_t uci_radar_start(uint32_t session_handle);

/* Stop a radar session */
uci_status_t uci_radar_stop(uint32_t session_handle);

/* Deinitialize a radar session */
uci_status_t uci_radar_deinit(uint32_t session_handle);

/* ========================================================================== */
/*  Convenience — Presence Detection with AoA                                 */
/* ========================================================================== */

/* Configure radar for presence detection with AoA in one call.
 * Sets RADAR_MODE=0x01 (medium distance, required for OCPD),
 * enables presence detection with distance+AoA (mode bits 0+1),
 * configures both RX antennas, and uses default 50ms RFRI. */
uci_status_t uci_radar_configure_presence_aoa(
    uint32_t session_handle,
    uint8_t  channel,               /* 5 or 9 */
    uint8_t  ant_tx_id,
    uint8_t  ant_rxb_id,            /* Primary RX */
    uint8_t  ant_rxc_id,            /* Second RX for AoA (must be non-zero) */
    uint16_t distance_min_cm,
    uint16_t distance_max_cm,
    int8_t   angle_min_deg,         /* -90 to +90 */
    int8_t   angle_max_deg          /* -90 to +90 */
);

/* ========================================================================== */
/*  RADAR_RX_NTF Parsing                                                      */
/* ========================================================================== */

/* Identify the data type from a RADAR_RX_NTF payload.
 * payload: raw notification payload (after UCI header), len: payload length */
uci_radar_data_type_t uci_radar_get_ntf_type(const uint8_t *payload, uint16_t len);

/* Parse CIR samples (data type 0x00).
 * Extracts metadata from TAP[0]-TAP[7] and returns a pointer to CIR data.
 * cir_data_out: pointer into payload at first CIR TAP (no copy) */
uci_status_t uci_radar_parse_cir_ntf(
    const uint8_t *payload, uint16_t len,
    uci_radar_cir_metadata_t *metadata,
    const uint8_t **cir_data_out,
    uint16_t *num_cir_taps
);

/* Parse presence detection result (data type 0x01) */
uci_status_t uci_radar_parse_presence_ntf(
    const uint8_t *payload, uint16_t len,
    uci_radar_presence_ntf_t *result
);

/* Parse antenna isolation report (data type 0x20) */
uci_status_t uci_radar_parse_ant_isolation_ntf(
    const uint8_t *payload, uint16_t len,
    uci_radar_ant_isolation_ntf_t *result
);

/* Extract a single complex CIR TAP value (16-bit resolution).
 * cir_data: pointer to first TAP byte (from uci_radar_parse_cir_ntf)
 * tap_index: 0-based index into CIR data TAPs (after metadata TAPs) */
void uci_radar_get_cir_tap(const uint8_t *cir_data, uint16_t tap_index,
                            int16_t *real, int16_t *imag);

/* ========================================================================== */
/*  Runtime Parameter Updates (while session is active)                       */
/* ========================================================================== */

/* Update RX gain during active radar session */
uci_status_t uci_radar_set_rx_gain(uint32_t session_handle,
                                    const uci_radar_rx_gain_t *gain);

/* Update RFRI during active radar session */
uci_status_t uci_radar_set_rfri(uint32_t session_handle,
                                 const uci_radar_rfri_t *rfri);

/* Update CIR start offset during active session */
uci_status_t uci_radar_set_cir_start_offset(uint32_t session_handle,
                                              const uci_radar_cir_start_offset_t *offset);

/* Update presence detection config during active session */
uci_status_t uci_radar_update_presence_cfg(uint32_t session_handle,
                                            const uci_radar_presence_cfg_t *cfg);

/* ========================================================================== */
/*  Antenna Isolation Test                                                    */
/* ========================================================================== */

/* Run antenna isolation test (RADAR_MODE=0x20).
 * Creates a temporary session, runs one measurement, parses result, deinits.
 * Blocks until result is received or timeout. */
uci_status_t uci_radar_run_ant_isolation_test(
    uint8_t channel, uint8_t ant_tx_id, uint8_t ant_rx_id,
    uci_radar_ant_isolation_ntf_t *result
);

#endif /* UCI_RADAR_H */

/*
 * uci_core.h
 *
 * UCI core engine: packet construction/parsing, synchronous command/response,
 * fragmentation reassembly, and notification dispatch.
 */

#ifndef UCI_CORE_H
#define UCI_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "uci_defs.h"
#include "uci_types.h"

/* Initialize the UCI core (call after uci_transport_init) */
void uci_core_init(void);

/* Register a callback for asynchronous notifications (RADAR_RX_NTF, SESSION_STATUS_NTF, etc.) */
void uci_core_register_ntf_callback(uci_ntf_callback_t callback);

/* Send a UCI command and wait synchronously for the matching response.
 * gid/oid:       command group and opcode
 * payload:       command payload (may be NULL if payload_len == 0)
 * payload_len:   payload byte count
 * rsp_payload:   output buffer for response payload (may be NULL if not needed)
 * rsp_len:       in: max buffer size, out: actual response payload length
 * timeout_ms:    max time to wait for response
 * Returns: status byte from response, or UCI_STATUS_TIMEOUT / UCI_STATUS_TRANSPORT_ERROR */
uci_status_t uci_core_send_command(
    uint8_t gid, uint8_t oid,
    const uint8_t *payload, uint16_t payload_len,
    uint8_t *rsp_payload, uint16_t *rsp_len,
    uint32_t timeout_ms
);

/* Poll for and process one incoming UCI packet from SR250.
 * - If a response is pending, stores it for uci_core_send_command.
 * - If a notification arrives, dispatches to the registered callback.
 * Call this from the main loop. */
void uci_core_process(void);

/* Get current device state (updated from CORE_DEVICE_STATUS_NTF) */
uint8_t uci_core_get_device_state(void);

/* ── Header Construction / Parsing Helpers ── */

/* Build a 4-byte UCI header into buf[0..3] */
void uci_build_header(uint8_t *buf, uint8_t mt, uint8_t pbf,
                       uint8_t gid, uint8_t oid, uint16_t payload_len);

/* Parse a 4-byte UCI header from buf[0..3] */
void uci_parse_header(const uint8_t *buf, uci_header_t *hdr);

#endif /* UCI_CORE_H */

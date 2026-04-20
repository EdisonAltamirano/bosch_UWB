/*
 * uci_transport.h
 *
 * SPI transport layer for UCI communication with SR250.
 * Implements direction-byte framing per UWBIoT Middleware Guide §3.1.
 *
 * Write transaction:  [0x00 (dir)] [UCI header + payload]
 * Read transaction:   Phase 1: [0xFF (dir)] → [echo + 4-byte header]
 *                     Phase 2: [0xFF (dir)] → [echo + payload]
 */

#ifndef UCI_TRANSPORT_H
#define UCI_TRANSPORT_H

#include <stdint.h>
#include <stdbool.h>
#include "uci_defs.h"
#include "uci_types.h"

/* Initialize the SPI transport layer (enables DWT cycle counter for µs delays) */
void uci_transport_init(void);

/* Send a raw UCI packet to SR250 (prepends direction byte 0x00)
 * data: pointer to complete UCI packet (header + payload), len: total bytes */
uci_status_t uci_transport_send(const uint8_t *data, uint16_t len);

/* Receive a UCI packet from SR250 using two-phase read.
 * buffer:  output buffer (receives header + payload)
 * out_len: total bytes written (header + payload)
 * max_len: size of buffer */
uci_status_t uci_transport_receive(uint8_t *buffer, uint16_t *out_len, uint16_t max_len);

/* Check if SR250 has data ready (polls INT line, active high) */
bool uci_transport_data_available(void);

/* Hardware reset of SR250 (RST_N low 10ms, release, wait 100ms for boot) */
void uci_transport_hw_reset(void);

/* Microsecond delay using DWT cycle counter (480MHz CPU) */
void uci_delay_us(uint32_t us);

#endif /* UCI_TRANSPORT_H */

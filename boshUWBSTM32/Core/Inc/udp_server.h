/*
 * udp_server.h
 *
 *  Created on: Mar 24, 2026
 *      Author: RAU1PAL
 */

#ifndef INC_UDP_SERVER_H_
#define INC_UDP_SERVER_H_

#include "lwip/ip_addr.h"

#define NUCLEO_ETH_PORT	37249	/* Single digit addition of decimal ASCII values representing BOSCH */

typedef struct {
    uint32_t send_attempts;
    uint32_t send_successes;
    uint32_t send_failures;
    uint32_t pbuf_alloc_failures;
    int32_t last_err;
    uint16_t last_port;
    uint16_t last_len;
} udp_tx_debug_stats_t;

void nucleo_udp_init(void);
uint8_t nucleo_udp_send(u16_t port, u8_t *payload, u16_t payload_length);
void nucleo_udp_set_destination(const ip_addr_t *addr);
const udp_tx_debug_stats_t *nucleo_udp_get_tx_stats(void);

typedef struct {
	uint8_t cmd_id;	// 1: Set radar configuration index and duration of run
	uint8_t setting_idx;
	uint32_t duration_in_ms;
}udp_input_cmd_t;

#endif /* INC_UDP_SERVER_H_ */

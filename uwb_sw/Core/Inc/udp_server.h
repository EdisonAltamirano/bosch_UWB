/*
 * udp_server.h
 *
 *  Created on: Mar 24, 2026
 *      Author: RAU1PAL
 */

#ifndef INC_UDP_SERVER_H_
#define INC_UDP_SERVER_H_

#define NUCLEO_ETH_PORT	37249	/* Single digit addition of decimal ASCII values representing BOSCH */

void nucleo_udp_init(void);
uint8_t nucleo_udp_send(u16_t port, u8_t *payload, u16_t payload_length);

typedef struct {
	uint8_t cmd_id;	// 1: Set radar configuration index and duration of run
	uint8_t setting_idx;
	uint32_t duration_in_ms;
}udp_input_cmd_t;

#endif /* INC_UDP_SERVER_H_ */

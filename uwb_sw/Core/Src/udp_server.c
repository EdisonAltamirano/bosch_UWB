/*
 * udp_server.c
 *
 *  Created on: Mar 24, 2026
 *      Author: RAU1PAL
 */

#include "lwip/pbuf.h"  // Provides structures and functions for packet buffers.
#include "lwip/udp.h"   // Includes UDP-specific functionality.

#include "stdio.h"      // Standard input/output library.
#include "udp_server.h" // UDP server header file
#include "uci/uci_types.h"
#include <string.h>


extern uint8_t IP_ADDRESS[4];
ip_addr_t destination_ip_addr;
struct udp_pcb *send_upcb;

extern volatile uci_main_control_states_t main_control_state;
extern volatile uint8_t 	user_radar_config_idx;
extern volatile uint32_t	user_radar_session_duration_ms;

void udp_receive_callback(void *arg, struct udp_pcb *upcb,
                          struct pbuf *p, const ip_addr_t *addr, u16_t port);

static inline void put_le16(uint8_t *buf, uint16_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)(val >> 8);
}

static inline uint32_t get_le32(const uint8_t *buf)
{
    return (uint32_t)buf[0] |
           ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) |
           ((uint32_t)buf[3] << 24);
}

static inline void put_le32(uint8_t *buf, uint32_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

void nucleo_udp_init(void)
{
    struct udp_pcb *upcb;
    err_t err;

    /* Debug print */
    printf("Inferred IP address: %d.%d.%d.%d\n", IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
    destination_ip_addr.addr = 0x6601A8C0;

    /* 1. Create a new UDP control block */
    upcb = udp_new();
    send_upcb = udp_new();
    udp_connect(send_upcb, &destination_ip_addr, 20000);

    /* 2. Bind the upcb to the STM32 static IP and port 7 */
    ip_addr_t nucleo_board_ip_addr;
    IP_ADDR4(&nucleo_board_ip_addr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);

    err = udp_bind(upcb, &nucleo_board_ip_addr, NUCLEO_ETH_PORT);

    /* 3. Register the receive callback if bind succeeded */
    if(err == ERR_OK)
    {
        udp_recv(upcb, udp_receive_callback, NULL);
    }
    else
    {
        udp_remove(upcb);
    }
}

void udp_receive_callback(void *arg, struct udp_pcb *upcb,
                           struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	udp_input_cmd_t recv_cmd;
	udp_input_ack_cmd_t ack_cmd;
	udp_err_report_cmd_t err_report_cmd;

    /* Build the reply string using the received payload */
	recv_cmd.cmd_id = *(uint8_t *)p->payload;
	if (recv_cmd.cmd_id == CMD_START_SESSION) {
		recv_cmd.setting_idx = *(uint8_t *)(p->payload+1);
		recv_cmd.duration_in_ms = get_le32((uint8_t *)(p->payload + 2));

		ack_cmd.cmd_id = CMD_UDP_ETH_ACK;
		ack_cmd.recv_cmd_id = recv_cmd.cmd_id;

		nucleo_udp_send(NUCLEO_REMOTE_PORT, CMD_UDP_ETH_ACK, (uint8_t *)&ack_cmd, 2);

		user_radar_config_idx = recv_cmd.setting_idx;
		user_radar_session_duration_ms = recv_cmd.duration_in_ms;
		main_control_state = UCI_INIT_SYSTEM;
	}
	else {
		err_report_cmd.cmd_id = CMD_ERROR_REPORT;
		err_report_cmd.err_id = ERR_UNKNOWN_USER_CMD_RECEIVED;
		nucleo_udp_send(NUCLEO_REMOTE_PORT, CMD_UDP_ETH_ACK, (uint8_t *)&err_report_cmd, 2);
	}

    pbuf_free(p);
}


uint8_t nucleo_udp_send(u16_t port, u8_t cmd_id, const u8_t *payload, u16_t payload_length)
{
	uint8_t send_status = 1;

	switch (cmd_id) {
		/* Cases where the payload length will not exceed MAX_DATA_PER_ETH_PACKET */
		case CMD_UDP_ETH_ACK:
		case CMD_ERROR_REPORT:
			struct pbuf *txBuf;
			txBuf = pbuf_alloc(PBUF_TRANSPORT, payload_length, PBUF_RAM);
			if (txBuf) {
				/* Copy the reply data into the pbuf */
				pbuf_take(txBuf, payload, payload_length);

				/* Set the client as the destination and send */

				udp_send(send_upcb, txBuf);
				send_status = 0;
			}
			else {
				send_status = 1;
			}
			pbuf_free(txBuf);
			break;
		case CMD_CIR_REPORT:
			udp_cir_fragment_t cir_fragment_pkt;
			uint16_t next_start_offset = 0;
			uint16_t remaining_length = payload_length;
			uint16_t current_length = 0;

			while (remaining_length > 0) {
				struct pbuf *txBuf;

				cir_fragment_pkt.cmd_id = cmd_id;
				if (remaining_length > MAX_DATA_PER_ETH_PACKET) {
					cir_fragment_pkt.last_fragment = 0x00;
					put_le16((uint8_t *)&cir_fragment_pkt.data_len, MAX_DATA_PER_ETH_PACKET);
					memcpy(&cir_fragment_pkt.data[0], payload + next_start_offset, MAX_DATA_PER_ETH_PACKET);
					next_start_offset += MAX_DATA_PER_ETH_PACKET;
					remaining_length -= MAX_DATA_PER_ETH_PACKET;
					current_length = MAX_DATA_PER_ETH_PACKET;
				}
				else {
					cir_fragment_pkt.last_fragment = 0x01;
					put_le16((uint8_t *)&cir_fragment_pkt.data_len, remaining_length);
					memcpy(&cir_fragment_pkt.data[0], payload + next_start_offset, remaining_length);
					next_start_offset = 0;
					current_length = remaining_length;
					remaining_length = 0;
				}

				// Add header length to current length
				current_length += 4;

				txBuf = pbuf_alloc(PBUF_TRANSPORT, current_length, PBUF_RAM);
			    if (txBuf) {
					/* Copy the reply data into the pbuf */
					pbuf_take(txBuf, (uint8_t *)&cir_fragment_pkt, current_length);

					/* Set the client as the destination and send */

					udp_send(send_upcb, txBuf);
					send_status = 0;
			    }
			    else {
			    	send_status = 1;
			    }
			    pbuf_free(txBuf);
			}
			break;
	}
    return send_status;
}


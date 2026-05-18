/*
 * udp_server.c
 *
 *  Created on: Mar 24, 2026
 *      Author: RAU1PAL
 */

#include "lwip/pbuf.h"  // Provides structures and functions for packet buffers.
#include "lwip/udp.h"   // Includes UDP-specific functionality.

#include "main.h"
#include "stdio.h"      // Standard input/output library.
#include "string.h"
#include "udp_server.h" // UDP server header file
#include "uwb_udp_protocol.h"

extern uint8_t IP_ADDRESS[4];
ip_addr_t destination_ip_addr;
struct udp_pcb *send_upcb;
static udp_tx_debug_stats_t s_udp_tx_stats;

void udp_receive_callback(void *arg, struct udp_pcb *upcb,
                          struct pbuf *p, const ip_addr_t *addr, u16_t port);

void nucleo_udp_init(void)
{
    struct udp_pcb *upcb;
    err_t err;

    memset(&s_udp_tx_stats, 0, sizeof(s_udp_tx_stats));
    s_udp_tx_stats.last_err = ERR_OK;

    /* Debug print */
    printf("Inferred IP address: %d.%d.%d.%d\n", IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
    destination_ip_addr.addr = 0x6601A8C0;

    /* 1. Create a new UDP control block */
    upcb = udp_new();
    send_upcb = udp_new();

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
    static uint8_t rx_buf[512];
    u16_t copy_len = p->tot_len;

    nucleo_udp_set_destination(addr);

    if (copy_len <= sizeof(rx_buf)) {
        pbuf_copy_partial(p, rx_buf, copy_len, 0);
        uwb_udp_protocol_handle_packet(rx_buf, copy_len, addr, port);
    } else {
        printf("UDP command too large: %u bytes\n\r", copy_len);
    }

    pbuf_free(p);
}

uint8_t nucleo_udp_send(u16_t port, u8_t *payload, u16_t payload_length)
{
	struct pbuf *txBuf;
    uint32_t start_tick;
    uint32_t end_tick;
	uint8_t send_status = 1;

    s_udp_tx_stats.send_attempts++;
    s_udp_tx_stats.last_port = port;
    s_udp_tx_stats.last_len = payload_length;
    start_tick = HAL_GetTick();

//	upcb = udp_new();

    /* Allocate a pbuf for the reply */
    txBuf = pbuf_alloc(PBUF_TRANSPORT, payload_length, PBUF_RAM);

    if (txBuf) {
		/* Copy the reply data into the pbuf */
		pbuf_take(txBuf, payload, payload_length);

		/* Set the client as the destination and send */
		err_t err = udp_sendto(send_upcb, txBuf, &destination_ip_addr, port);
        end_tick = HAL_GetTick();
        s_udp_tx_stats.last_err = err;
		if (err != ERR_OK) {
		    printf("[UDP] udp_sendto failed: err=%d port=%u len=%u dt=%lums\n\r",
		           (int)err, port, payload_length,
                   (unsigned long)(end_tick - start_tick));
            s_udp_tx_stats.send_failures++;
            pbuf_free(txBuf);
		    send_status = 1;
		} else {
            s_udp_tx_stats.send_successes++;
            pbuf_free(txBuf);
		    send_status = 0;
		}
    }
    else {
        end_tick = HAL_GetTick();
        printf("[UDP] pbuf_alloc failed: len=%u dt=%lums\n\r",
               (unsigned)payload_length,
               (unsigned long)(end_tick - start_tick));
        s_udp_tx_stats.pbuf_alloc_failures++;
        s_udp_tx_stats.send_failures++;
        s_udp_tx_stats.last_err = ERR_MEM;
    	send_status = 1;
    }

    /* Disconnect so the server is ready for the next client */
//    udp_disconnect(upcb);
//    udp_remove(upcb);

    return send_status;
}

void nucleo_udp_set_destination(const ip_addr_t *addr)
{
    if (addr != NULL) {
        destination_ip_addr = *addr;
    }
}

const udp_tx_debug_stats_t *nucleo_udp_get_tx_stats(void)
{
    return &s_udp_tx_stats;
}


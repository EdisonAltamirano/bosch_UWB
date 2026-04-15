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

extern uint8_t IP_ADDRESS[4];
ip_addr_t destination_ip_addr;
struct udp_pcb *send_upcb;

void udp_receive_callback(void *arg, struct udp_pcb *upcb,
                          struct pbuf *p, const ip_addr_t *addr, u16_t port);

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
//    struct pbuf *txBuf;

    /* Build the reply string using the received payload */
    char buf[100];
    int len = sprintf(buf, "Hello %s From UDP SERVER\n", (char *)p->payload);

//    if (destination_ip_addr.addr == 0) {
//    	destination_ip_addr = *addr;
//    }
//
//    /* Allocate a pbuf for the reply */
//    txBuf = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
//
//    /* Copy the reply data into the pbuf */
//    pbuf_take(txBuf, buf, len);
//
//    /* Set the client as the destination and send */
//    udp_connect(upcb, addr, 20000);
//    udp_send(upcb, txBuf);
//
//    /* Disconnect so the server is ready for the next client */
//    udp_disconnect(upcb);
//
//    /* Free both buffers - never skip this */
//    pbuf_free(txBuf);
    pbuf_free(p);
}

uint8_t nucleo_udp_send(u16_t port, u8_t *payload, u16_t payload_length)
{
	struct pbuf *txBuf;

	uint8_t send_status = 1;

//	upcb = udp_new();

    /* Allocate a pbuf for the reply */
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

    /* Disconnect so the server is ready for the next client */
//    udp_disconnect(upcb);
//    udp_remove(upcb);

    /* Free both buffers - never skip this */
    pbuf_free(txBuf);

    return send_status;
}


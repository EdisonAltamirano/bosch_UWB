/*
 * udp_server.h
 *
 *  Created on: Mar 24, 2026
 *      Author: RAU1PAL
 */

#ifndef INC_UDP_SERVER_H_
#define INC_UDP_SERVER_H_

#define NUCLEO_ETH_PORT			37249	/* Single digit addition of decimal ASCII values representing BOSCH */
#define NUCLEO_REMOTE_PORT		20000
#define MAX_DATA_PER_ETH_PACKET	1184 /* Pbuf setting in CubeMX is 592bytes. Max 2 Pbufs per packet */

/* Valid cmd_ids */
#define CMD_START_SESSION			0x01
#define CMD_CIR_REPORT				0x02
#define CMD_UDP_ETH_ACK				0x03
#define CMD_ERROR_REPORT			0x04
#define CMD_SYSTEM_COMMAND			0x05
#define CMD_SYSTEM_STATE			0x06

/* Error ids for udp_err_report_cmd_t */
#define ERR_INIT_UWB_SUBSYSTEM_FAILED			0x03
#define ERR_INIT_UWB_RADAR_FAILED				0x05
#define ERR_CONFIG_UWB_RADAR_FAILED				0x07
#define ERR_START_UWB_RADAR_FAILED				0x09
#define ERR_UWB_RADAR_SESSION_STOP_FAILED		0x0D
#define ERR_UWB_RADAR_SESSION_DEINIT_FAILED		0x0E
#define ERR_UNKNOWN_USER_CMD_RECEIVED			0x0F
#define ERR_UNKNOWN_SYS_CMD_RECEIVED			0x10

/* System state commands */
#define CMD_SYSTEM_RESET						0x01
#define CMD_SYSTEM_STATE_OK						0x00
#define CMD_SYSTEM_STATE_NOK					0x01
#define CMD_SYSTEM_STATE_RADAR_SESSION_DONE		0x02

void nucleo_udp_init(void);
uint8_t nucleo_udp_send(u16_t port, u8_t cmd_id, const u8_t *payload, u16_t payload_length);

/* Only input structure expected from the user side
 * Contains information regarding the radar preset config
 * to use and the duration of the radar session in ms
 */
typedef struct {
	uint8_t cmd_id;	// 0x01: Set radar configuration index and duration of run
	uint8_t setting_idx;
	uint32_t duration_in_ms;
}udp_input_cmd_t;

/* Input from the user side is acknowledged by sending
 * a UDP packet with command id CMD_UDP_ETH_ACK and the most
 * recent cmd_id received from the user side
 */
typedef struct {
	uint8_t cmd_id;	// 0x03: Ack for received cmd
	uint8_t recv_cmd_id;
}udp_input_ack_cmd_t;

/* CIR report packet as per the details available in the
 * system overview slide deck
 */
typedef struct {
	uint8_t cmd_id; // 0x02: CIR report fragment
	uint8_t last_fragment; // 0x00 --> no, 0x01 --> yes
	uint16_t data_len;
	uint8_t data[MAX_DATA_PER_ETH_PACKET];
}udp_cir_fragment_t;

/* Any errors encountered during execution of the
 * main controller state machine is reported back to the
 * user side
 */
typedef struct {
	uint8_t cmd_id;	// 0x04: Error report
	uint8_t err_id;
}udp_err_report_cmd_t;

#endif /* INC_UDP_SERVER_H_ */

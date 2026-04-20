#ifndef INC_UWB_UDP_PROTOCOL_H_
#define INC_UWB_UDP_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>
#include "lwip/ip_addr.h"

#define UWB_UDP_MAGIC_0                 0x55
#define UWB_UDP_MAGIC_1                 0x57
#define UWB_UDP_PROTOCOL_VERSION        0x01

#define UWB_MSG_SET_CONFIG_FULL         0x01
#define UWB_MSG_SET_PARAMS_PARTIAL      0x02
#define UWB_MSG_START_RADAR             0x03
#define UWB_MSG_STOP_RADAR              0x04
#define UWB_MSG_ACK                     0x05
#define UWB_MSG_ERROR                   0x06
#define UWB_MSG_RADAR_FRAME             0x07

#define UWB_TAG_CHANNEL_NUMBER          0x0001
#define UWB_TAG_PREAMBLE_CODE_INDEX     0x0002
#define UWB_TAG_ANTENNAS_CONFIG_RX      0x0003
#define UWB_TAG_ANTENNAS_CONFIG_TX      0x0004
#define UWB_TAG_RADAR_MODE              0x0005
#define UWB_TAG_RADAR_SINGLE_FRAME_NTF  0x0006
#define UWB_TAG_RADAR_RFRI              0x0007
#define UWB_TAG_RADAR_CIR_NUM_SAMPLES   0x0008
#define UWB_TAG_RADAR_RX_GAIN           0x0009
#define UWB_TAG_RADAR_CIR_START_OFFSET  0x000A
#define UWB_TAG_RADAR_PERFORMANCE       0x000B
#define UWB_TAG_RADAR_DRIFT_COMP        0x000C
#define UWB_TAG_RADAR_PRESENCE_CFG      0x000D

#define UWB_ERR_INVALID_MAGIC           0x0001
#define UWB_ERR_INVALID_VERSION         0x0002
#define UWB_ERR_INVALID_LENGTH          0x0003
#define UWB_ERR_INVALID_TLV             0x0004
#define UWB_ERR_MISSING_REQUIRED_FIELD  0x0005
#define UWB_ERR_INVALID_STATE           0x0006
#define UWB_ERR_UCI_COMMAND_FAILED      0x0007
#define UWB_ERR_UNSUPPORTED_TAG         0x0008

#define UWB_RADAR_FRAME_PORT           20000
#define UWB_SESSION_ID                 0x11223344UL

void uwb_udp_protocol_init(void);
void uwb_udp_protocol_handle_packet(const uint8_t *data, uint16_t len,
                                     const ip_addr_t *addr, uint16_t port);
void uwb_udp_protocol_send_radar_frame(const uint8_t *raw_payload, uint16_t raw_len);

#endif /* INC_UWB_UDP_PROTOCOL_H_ */

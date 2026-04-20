/*
 * uci_defs.h
 *
 * UCI protocol constants derived from:
 *   - FiRa UCI Technical Specification v3.0.0
 *   - NXP SR250 UCI Specification v2.0.23
 */

#ifndef UCI_DEFS_H
#define UCI_DEFS_H

#include <stdint.h>

/* ========================================================================== */
/*  UCI Packet Header                                                         */
/* ========================================================================== */

#define UCI_HDR_SIZE                4
#define UCI_MAX_PAYLOAD_SIZE        255
#define UCI_MAX_EXT_PAYLOAD_SIZE    2048

/* Message Type (MT) — bits [7:5] of octet 0 */
#define UCI_MT_DATA                 0x00
#define UCI_MT_CMD                  0x01
#define UCI_MT_RSP                  0x02
#define UCI_MT_NTF                  0x03

/* Packet Boundary Flag (PBF) — bit 4 of octet 0 */
#define UCI_PBF_COMPLETE            0x00
#define UCI_PBF_SEGMENT             0x01

/* Data Packet Format (DPF) — bits [3:0] of octet 0 for data packets */
#define UCI_DPF_DATA_SND            0x01
#define UCI_DPF_DATA_RCV            0x02

/* NXP extended length indicator — bit 7 of octet 1 */
#define UCI_EXT_LEN_BIT             0x80

/* Direction bytes for SPI transport */
#define UCI_DIR_BYTE_WRITE          0x00
#define UCI_DIR_BYTE_READ           0xFF

/* ========================================================================== */
/*  Group IDs (GID) — bits [3:0] of octet 0                                  */
/* ========================================================================== */

#define UCI_GID_CORE                0x00
#define UCI_GID_SESSION_CONFIG      0x01
#define UCI_GID_SESSION_CONTROL     0x02
#define UCI_GID_VENDOR_NXP_PROP		0x09
#define UCI_GID_VENDOR_NXP_PROP_IOT     0x0A    /* IOT Proprietary (radar NTFs) */
#define UCI_GID_TEST                0x0D
#define UCI_GID_VENDOR_E            0x0E    /* Customer Proprietary 1 */
#define UCI_GID_VENDOR_F            0x0F    /* Customer Proprietary 2 */

/* ========================================================================== */
/*  Opcode IDs (OID) — bits [5:0] of octet 1                                 */
/* ========================================================================== */

/* GID 0x00: Core */
#define UCI_OID_CORE_DEVICE_RESET           0x00
#define UCI_OID_CORE_DEVICE_STATUS_NTF      0x01
#define UCI_OID_CORE_GET_DEVICE_INFO        0x02
#define UCI_OID_CORE_GET_CAPS_INFO          0x03
#define UCI_OID_CORE_SET_CONFIG             0x04
#define UCI_OID_CORE_GET_CONFIG             0x05
#define UCI_OID_CORE_GENERIC_ERROR_NTF      0x07
#define UCI_OID_CORE_QUERY_TIMESTAMP        0x08

/* GID 0x01: Session Config */
#define UCI_OID_SESSION_INIT                0x00
#define UCI_OID_SESSION_DEINIT              0x01
#define UCI_OID_SESSION_STATUS_NTF          0x02
#define UCI_OID_SESSION_SET_APP_CONFIG      0x03
#define UCI_OID_SESSION_GET_APP_CONFIG      0x04
#define UCI_OID_SESSION_GET_COUNT           0x05
#define UCI_OID_SESSION_GET_STATE           0x06
#define UCI_OID_SESSION_UPDATE_MCAST_LIST   0x07

/* GID 0x02: Session Control */
#define UCI_OID_SESSION_START               0x00
#define UCI_OID_SESSION_STOP                0x01
#define UCI_OID_SESSION_GET_RANGING_COUNT   0x03
#define UCI_OID_SESSION_DATA_CREDIT_NTF     0x04
#define UCI_OID_SESSION_DATA_XFER_STATUS_NTF 0x05

/* GID 0x0A: IOT Proprietary */
#define UCI_OID_RADAR_RX_NTF                0x0A

/* GID 0x0E: Customer Proprietary 1 */
#define UCI_OID_NXP_CORE_DEVICE_INIT        0x00
#define UCI_OID_NXP_DBG_DATA_LOGGER_NTF     0x01
#define UCI_OID_NXP_DBG_GET_ERROR_LOG       0x02
#define UCI_OID_NXP_SE_GET_BINDING_COUNT    0x03
#define UCI_OID_NXP_SE_COMM_ERROR_NTF       0x05
#define UCI_OID_NXP_BINDING_STATUS_NTF      0x06
#define UCI_OID_NXP_SCHEDULER_STATUS_NTF    0x07
#define UCI_OID_NXP_SESSION_KDF_NTF         0x08
#define UCI_OID_NXP_QUERY_TEMPERATURE       0x0B
#define UCI_OID_NXP_WLAN_COEX_NTF          0x13
#define UCI_OID_NXP_PROP_GEN_DEBUG_NTF     0x18
#define UCI_OID_NXP_TIMESYNC_NTF           0x1D
#define UCI_OID_NXP_DATA_DOWNLOAD_START    0x24
#define UCI_OID_NXP_DATA_DOWNLOAD          0x25
#define UCI_OID_NXP_DATA_DOWNLOAD_STOP     0x26
#define UCI_OID_NXP_DATA_DOWNLOAD_QUERY    0x27
#define UCI_OID_NXP_DATA_DOWNLOAD_DELETE   0x28
#define UCI_OID_NXP_DATA_DOWNLOAD_COMPLETE 0x29

/* GID 0x0F: Customer Proprietary 2 */
#define UCI_OID_NXP_SET_VENDOR_APP_CONFIG   0x00
#define UCI_OID_NXP_GET_ALL_SESSIONS        0x02
#define UCI_OID_NXP_GET_VENDOR_APP_CONFIG   0x03
#define UCI_OID_NXP_DO_CALIBRATION          0x20
#define UCI_OID_NXP_SET_CALIBRATION         0x21
#define UCI_OID_NXP_GET_CALIBRATION         0x22
#define UCI_OID_NXP_SET_SECURE_CALIBRATION  0x23

/* ========================================================================== */
/*  Status Codes                                                              */
/* ========================================================================== */

/* Generic */
#define UCI_STATUS_OK                       0x00
#define UCI_STATUS_REJECTED                 0x01
#define UCI_STATUS_FAILED                   0x02
#define UCI_STATUS_SYNTAX_ERROR             0x03
#define UCI_STATUS_INVALID_PARAM            0x04
#define UCI_STATUS_INVALID_RANGE            0x05
#define UCI_STATUS_INVALID_MSG_SIZE         0x06
#define UCI_STATUS_UNKNOWN_GID              0x07
#define UCI_STATUS_UNKNOWN_OID              0x08
#define UCI_STATUS_READ_ONLY                0x09
#define UCI_STATUS_MESSAGE_RETRY            0x0A
#define UCI_STATUS_UNKNOWN                  0x0B
#define UCI_STATUS_NOT_APPLICABLE           0x0C

/* Session-specific */
#define UCI_STATUS_SESSION_NOT_EXIST        0x11
#define UCI_STATUS_SESSION_ACTIVE           0x13
#define UCI_STATUS_MAX_SESSIONS_EXCEEDED    0x14
#define UCI_STATUS_SESSION_NOT_CONFIGURED   0x15
#define UCI_STATUS_ACTIVE_SESSIONS_ONGOING  0x16
#define UCI_STATUS_MULTICAST_LIST_FULL      0x17
#define UCI_STATUS_INVALID_SLOT_ALLOC       0x18
#define UCI_STATUS_INVALID_SLOT_DURATION    0x19
#define UCI_STATUS_INIT_TIME_TOO_OLD        0x1A
#define UCI_STATUS_OK_NEGATIVE_DISTANCE     0x1B

/* Ranging */
#define UCI_STATUS_RANGING_TX_FAILED        0x20
#define UCI_STATUS_RANGING_RX_TIMEOUT       0x21
#define UCI_STATUS_RANGING_RX_PHY_DEC_FAIL  0x22
#define UCI_STATUS_RANGING_RX_PHY_TOA_FAIL  0x23
#define UCI_STATUS_RANGING_RX_PHY_STS_FAIL  0x24
#define UCI_STATUS_RANGING_RX_MAC_DEC_FAIL  0x25
#define UCI_STATUS_RANGING_RX_MAC_IE_DEC_FAIL 0x26
#define UCI_STATUS_RANGING_RX_MAC_IE_MISSING  0x27

/* NXP Proprietary — Calibration */
#define UCI_STATUS_CALIBRATION_IN_PROGRESS  0x53
#define UCI_STATUS_THERMAL_RUNAWAY          0x54
#define UCI_STATUS_FEATURE_NOT_SUPPORTED    0x55
#define UCI_STATUS_CALIBRATION_NOT_CONFIGURED 0x57
#define UCI_STATUS_UNDERFLOW_TIMEOUT        0x58
#define UCI_STATUS_DEVICE_LOW_VBAT          0x59

/* NXP Proprietary — Radar */
#define UCI_STATUS_RADAR_CIR_MAX_TAP_EXCEEDED  0xB0
#define UCI_STATUS_RADAR_ANT_CONFIG_RX_NOT_OK  0xB1
#define UCI_STATUS_RADAR_PRESENCE_RANGE_EXCEEDED 0xB2
#define UCI_STATUS_RADAR_RX_GAIN_NOT_OK        0xB3
#define UCI_STATUS_RADAR_DRIFTCOMP_ANT_NOT_OK  0xB4
#define UCI_STATUS_RADAR_LOGATING_CALIB_INVALID 0xB6

/* NXP Proprietary — Calibration security */
#define UCI_STATUS_CALIB_FORBIDDEN          0xB0
#define UCI_STATUS_CALIB_NOT_AUTHORIZED     0xB1
#define UCI_STATUS_CALIB_INVALID_VERSION    0xB2
#define UCI_STATUS_CALIB_NO_PLATFORM_ID     0xB3

/* NXP Proprietary — SE */
#define UCI_STATUS_SE_NO_SE                 0x70
#define UCI_STATUS_SE_RECOVERY_FAIL         0x71
#define UCI_STATUS_SE_RECOVERY_SUCCESS      0x72
#define UCI_STATUS_SE_APDU_FAIL             0x73
#define UCI_STATUS_SE_AUTH_FAIL             0x74

/* Internal driver status (not from UCI spec) */
#define UCI_STATUS_TIMEOUT                  0xFE
#define UCI_STATUS_TRANSPORT_ERROR          0xFD

/* ========================================================================== */
/*  Device State (CORE_DEVICE_STATUS_NTF)                                     */
/* ========================================================================== */

#define UCI_DEVICE_STATUS_READY             0x01
#define UCI_DEVICE_STATUS_ACTIVE            0x02
#define UCI_DEVICE_STATUS_FATAL_ERROR       0xFD
#define UCI_DEVICE_STATUS_ERROR             0xFF

/* ========================================================================== */
/*  Session State (SESSION_STATUS_NTF)                                        */
/* ========================================================================== */

#define UCI_SESSION_STATE_INIT              0x00
#define UCI_SESSION_STATE_DEINIT            0x01
#define UCI_SESSION_STATE_ACTIVE            0x02
#define UCI_SESSION_STATE_IDLE              0x03

/* ========================================================================== */
/*  Session Types                                                             */
/* ========================================================================== */

#define UCI_SESSION_TYPE_RANGING            0x00
#define UCI_SESSION_TYPE_RANGING_DATA       0x01
#define UCI_SESSION_TYPE_DATA_TRANSFER      0x02
#define UCI_SESSION_TYPE_DEVICE_TEST        0xD0
#define UCI_SESSION_TYPE_RADAR              0xF0

/* ========================================================================== */
/*  Standard APP Configuration Parameter Tags (FiRa UCI Table 64)             */
/* ========================================================================== */

#define UCI_PARAM_DEVICE_TYPE               0x00
#define UCI_PARAM_RANGING_ROUND_USAGE       0x01
#define UCI_PARAM_STS_CONFIG                0x02
#define UCI_PARAM_MULTI_NODE_MODE           0x03
#define UCI_PARAM_CHANNEL_NUMBER            0x04
#define UCI_PARAM_NUM_OF_CONTROLEES         0x05
#define UCI_PARAM_DEVICE_MAC_ADDRESS        0x06
#define UCI_PARAM_DST_MAC_ADDRESS           0x07
#define UCI_PARAM_SLOT_DURATION             0x08
#define UCI_PARAM_RANGING_DURATION          0x09
#define UCI_PARAM_STS_INDEX                 0x0A
#define UCI_PARAM_MAC_FCS_TYPE              0x0B
#define UCI_PARAM_RANGING_ROUND_CONTROL     0x0C
#define UCI_PARAM_AOA_RESULT_REQ            0x0D
#define UCI_PARAM_SESSION_INFO_NTF_CONFIG   0x0E
#define UCI_PARAM_NEAR_PROXIMITY_CONFIG     0x0F
#define UCI_PARAM_FAR_PROXIMITY_CONFIG      0x10
#define UCI_PARAM_DEVICE_ROLE               0x11
#define UCI_PARAM_RFRAME_CONFIG             0x12
#define UCI_PARAM_RSSI_REPORTING            0x13
#define UCI_PARAM_PREAMBLE_CODE_INDEX       0x14
#define UCI_PARAM_SFD_ID                    0x15
#define UCI_PARAM_PSDU_DATA_RATE            0x16
#define UCI_PARAM_PREAMBLE_DURATION         0x17
#define UCI_PARAM_LINK_LAYER_MODE           0x18
#define UCI_PARAM_DATA_REPETITION_COUNT     0x19
#define UCI_PARAM_RANGING_TIME_STRUCT       0x1A
#define UCI_PARAM_SLOTS_PER_RR              0x1B
#define UCI_PARAM_AOA_BOUND_CONFIG          0x1D
#define UCI_PARAM_PRF_MODE                  0x1F
#define UCI_PARAM_SCHEDULE_MODE             0x22
#define UCI_PARAM_KEY_ROTATION              0x23
#define UCI_PARAM_KEY_ROTATION_RATE         0x24
#define UCI_PARAM_SESSION_PRIORITY          0x25
#define UCI_PARAM_MAC_ADDRESS_MODE          0x26
#define UCI_PARAM_NUMBER_OF_STS_SEGMENTS    0x29
#define UCI_PARAM_MAX_RR_RETRY              0x2A
#define UCI_PARAM_HOPPING_MODE              0x2C
#define UCI_PARAM_BLOCK_STRIDE_LENGTH       0x2D
#define UCI_PARAM_RESULT_REPORT_CONFIG      0x2E
#define UCI_PARAM_IN_BAND_TERMINATION_ATTEMPT 0x2F
#define UCI_PARAM_BPRF_PHR_DATA_RATE        0x30
#define UCI_PARAM_MAX_NUM_OF_MEASUREMENTS   0x32
#define UCI_PARAM_UWB_INITIATION_TIME       0x33
#define UCI_PARAM_RANGING_ROUND_HOPPING     0x35

/* ========================================================================== */
/*  Device Configuration Parameter Tags (CORE_SET_CONFIG)                     */
/* ========================================================================== */

#define UCI_CFG_DEVICE_STATE                0x00
#define UCI_CFG_LOW_POWER_MODE              0x01

/* ========================================================================== */
/*  NXP Extended Device Config Tags (0xE4/XX via CORE_SET_CONFIG)             */
/* ========================================================================== */

#define NXP_CFG_EXT_TAG_PREFIX              0xE4
#define NXP_CFG_DPD_WAKEUP_SRC             0x02
#define NXP_CFG_DPD_ENTRY_TIMEOUT           0x04
#define NXP_CFG_GPIO_USAGE_CONFIG           0x08
#define NXP_CFG_PDOA_CALIB_TABLE_DEFINE     0x5E
#define NXP_CFG_ANTENNA_RX_IDX_DEFINE       0x60
#define NXP_CFG_ANTENNAS_RX_PAIR_DEFINE     0x62
#define NXP_CFG_ANTENNA_TX_IDX_DEFINE       0x63
#define NXP_CFG_DEFAULT_VENDOR_APP_CONFIG   0x65

/* ========================================================================== */
/*  NXP Vendor APP Configuration Parameter Tags (SET_VENDOR_APP_CONFIG)       */
/* ========================================================================== */

#define NXP_PARAM_MAC_PAYLOAD_ENCRYPTION    0x00
#define NXP_PARAM_ANTENNAS_CONFIG_TX        0x02
#define NXP_PARAM_ANTENNAS_CONFIG_RX        0x03
#define NXP_PARAM_CIR_LOG_NTF              0x30
#define NXP_PARAM_PSDU_LOG_NTF             0x31
#define NXP_PARAM_CIR_CAPTURE_MODE         0x60
#define NXP_PARAM_RX_ANT_POLARIZATION      0x61
#define NXP_PARAM_SESSION_SYNC_ATTEMPTS    0x62
#define NXP_PARAM_SCHED_ATTEMPTS           0x63
#define NXP_PARAM_SCHED_STATUS_NTF         0x64
#define NXP_PARAM_TX_POWER_DELTA_FCC       0x65
#define NXP_PARAM_TX_POWER_TEMP_COMP       0x67
#define NXP_PARAM_ADAPTIVE_HOP_THRESHOLD   0x69
#define NXP_PARAM_AUTHENTICITY_TAG         0x6E
#define NXP_PARAM_RX_NBIC_CONFIG           0x6F
#define NXP_PARAM_MAC_CFG                  0x70
#define NXP_PARAM_RFRAME_LOG_NTF           0x7B
#define NXP_PARAM_TX_ADAPTIVE_PAYLOAD_POWER 0x7F
#define NXP_PARAM_ENABLE_FOV               0x84
#define NXP_PARAM_AZIMUTH_FOV              0x85
#define NXP_PARAM_SESSION_INFO_NTF_FILTER  0x86
#define NXP_PARAM_ENABLE_FOM               0x91

/* Radar-specific vendor app config tags */
#define NXP_RADAR_PARAM_MODE               0xA0
#define NXP_RADAR_PARAM_RX_GAIN            0xA4
#define NXP_RADAR_PARAM_SINGLE_FRAME_NTF   0xA5
#define NXP_RADAR_PARAM_CIR_NUM_SAMPLES    0xA7
#define NXP_RADAR_PARAM_CIR_START_OFFSET   0xA8
#define NXP_RADAR_PARAM_RFRI               0xA9
#define NXP_RADAR_PARAM_PRESENCE_DET_CFG   0xAA
#define NXP_RADAR_PARAM_PERFORMANCE        0xAD
#define NXP_RADAR_PARAM_DRIFT_COMPENSATION 0xB2
#define NXP_RADAR_PARAM_CFG               0xB3
#define NXP_RADAR_PARAM_LPRF_CALIB        0xB4

/* ========================================================================== */
/*  Calibration Parameter Tags (SET/GET_DEVICE_CALIBRATION)                   */
/* ========================================================================== */

#define NXP_CALIB_CHIP_CALIBRATION          0x00
#define NXP_CALIB_RF_CLK_ACCURACY           0x01
#define NXP_CALIB_RX_ANT_DELAY              0x02
#define NXP_CALIB_PDOA_OFFSET               0x03
#define NXP_CALIB_TX_POWER_PER_ANTENNA      0x04
#define NXP_CALIB_AOA_PHASEFLIP_ANTSPACING  0x05
#define NXP_CALIB_PLATFORM_ID               0x5C
#define NXP_CALIB_CONFIG_VERSION            0x5D
#define NXP_CALIB_MANUAL_TX_POW_CTRL        0x60
#define NXP_CALIB_AOA_ANTENNAS_PDOA         0x62
#define NXP_CALIB_TX_ANT_DELAY              0x64
#define NXP_CALIB_PDOA_MANUFACT_ZERO_OFFSET 0x65
#define NXP_CALIB_AOA_THRESHOLD_PDOA        0x66
#define NXP_CALIB_TX_TEMP_COMP_PER_ANT      0x67
#define NXP_CALIB_SNR_CONST_PER_ANT         0x68
#define NXP_CALIB_RSSI_CONST_PER_ANT        0x69

/* ========================================================================== */
/*  ANTENNAS_CONFIGURATION_RX Modes                                           */
/* ========================================================================== */

#define NXP_ANT_RX_MODE_TOA                 0x00
#define NXP_ANT_RX_MODE_AOA                 0x01
#define NXP_ANT_RX_MODE_RADAR              0x02
#define NXP_ANT_RX_MODE_TOA_RFM            0x03
#define NXP_ANT_RX_MODE_AOA_RFM            0x04

/* RX Port assignments */
#define NXP_RX_PORT_RXC						0x01
#define NXP_RX_PORT_RXB                     0x02
#define NXP_RX_PORT_RXA2                    0x03
#define NXP_RX_PORT_RXA1                    0x04

/* TX Port assignments */
#define NXP_TX_PORT_TRA1                    0x01
#define NXP_TX_PORT_TRA2                    0x02

/* ========================================================================== */
/*  Device Role                                                               */
/* ========================================================================== */

#define UCI_ROLE_RESPONDER                  0x00
#define UCI_ROLE_INITIATOR                  0x01
#define UCI_ROLE_ADVERTISER                 0x05
#define UCI_ROLE_OBSERVER                   0x06

/* ========================================================================== */
/*  Device Type                                                               */
/* ========================================================================== */

#define UCI_DEVICE_TYPE_CONTROLEE           0x00
#define UCI_DEVICE_TYPE_CONTROLLER          0x01

/* ========================================================================== */
/*  Multi-Node Mode                                                           */
/* ========================================================================== */

#define UCI_MULTI_NODE_ONE_TO_ONE           0x00
#define UCI_MULTI_NODE_ONE_TO_MANY          0x01

/* ========================================================================== */
/*  Ranging Round Usage                                                       */
/* ========================================================================== */

#define UCI_RANGING_SS_TWR_DEFERRED         0x01
#define UCI_RANGING_DS_TWR_DEFERRED         0x02
#define UCI_RANGING_SS_TWR_NONDEFERRED      0x03
#define UCI_RANGING_DS_TWR_NONDEFERRED      0x04

/* ========================================================================== */
/*  PRF Mode                                                                  */
/* ========================================================================== */

#define UCI_PRF_MODE_BPRF                   0x00
#define UCI_PRF_MODE_HPRF_124               0x01
#define UCI_PRF_MODE_HPRF_249               0x02
#define UCI_PRF_MODE_RADAR                  0x03

/* ========================================================================== */
/*  CORE_DEVICE_RESET_CMD reset config                                        */
/* ========================================================================== */

#define UCI_RESET_CONFIG_UWBS_RESET         0x00

/* ========================================================================== */
/*  Timing Constants (from reference code analysis)                           */
/* ========================================================================== */

#define UCI_SPI_TIMEOUT_MS                  1000
#define UCI_POST_WRITE_DELAY_US             80
#define UCI_PRE_READ_DELAY_US               500
#define UCI_POST_READ_DELAY_US              50
#define UCI_RESET_HOLD_MS                   10
#define UCI_POST_RESET_WAIT_MS              100
#define UCI_CMD_RESPONSE_TIMEOUT_MS         20000

/* ========================================================================== */
/*  Buffer Sizes                                                              */
/* ========================================================================== */

#define APP_MAX_SLOTS_PER_RR				10	// Ensure maximum slots per
												// ranging round is 10
#define APP_MAX_ACTIVE_RX_ANT				2	// Ensure maximum number of rx
												// antennas in radar modes is 2
#define UCI_RADAR_RX_NTF_INFO_BLOCK_SIZE	10	// 4 (Session handle) + status (1)
												// + radar data type (1) +
												// Num CIR blocks (2) +
												// Taps per CIR block (1)
												// + RFU (1)
#define UCI_MAX_CMD_BUF_SIZE    (UCI_HDR_SIZE + UCI_MAX_EXT_PAYLOAD_SIZE)
#define UCI_MAX_RSP_BUF_SIZE    (UCI_HDR_SIZE + UCI_RADAR_RX_NTF_INFO_BLOCK_SIZE + (APP_MAX_SLOTS_PER_RR * APP_MAX_ACTIVE_RX_ANT * UCI_MAX_EXT_PAYLOAD_SIZE))
#define UCI_MAX_TLV_VALUE_LEN   255

#endif /* UCI_DEFS_H */

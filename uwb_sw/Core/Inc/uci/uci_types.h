/*
 * uci_types.h
 *
 * Data structures for UCI protocol, radar, and ranging.
 */

#ifndef UCI_TYPES_H
#define UCI_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "uci_defs.h"

/* ========================================================================== */
/*  Common Types                                                              */
/* ========================================================================== */

typedef uint8_t uci_status_t;

/* ── UCI Packet Header (parsed) ── */
typedef struct {
    uint8_t  mt;
    uint8_t  pbf;
    uint8_t  gid;
    uint8_t  oid;
    uint16_t payload_len;
} uci_header_t;

/* ── TLV (Tag-Length-Value) ── */
typedef struct {
    uint16_t tag;
    uint8_t  len;
    uint8_t  value[UCI_MAX_TLV_VALUE_LEN];
} uci_tlv_t;

/* ── Notification Callback ── */
typedef void (*uci_ntf_callback_t)(uint8_t gid, uint8_t oid,
                                    const uint8_t *payload, uint16_t len);

/* ========================================================================== */
/*  Device Info                                                               */
/* ========================================================================== */

typedef struct {
    uint8_t  status;
    uint16_t uci_version;
    uint16_t mac_version;
    uint16_t phy_version;
    uint16_t uci_test_version;
    uint8_t  vendor_spec_info[128];
    uint8_t  vendor_spec_len;
} uci_device_info_t;

/* ========================================================================== */
/*  Radar Types                                                               */
/* ========================================================================== */

/* ── Radar Mode ── */
typedef enum {
    UCI_RADAR_MODE_MEDIUM_DISTANCE = 0x01,
    UCI_RADAR_MODE_CLOSE_DISTANCE  = 0x02,
    UCI_RADAR_MODE_FAR_DISTANCE    = 0x03,
    UCI_RADAR_MODE_SINGLE_PIN      = 0x04,
    UCI_RADAR_MODE_MEDIUM_STATIC   = 0x05,
    UCI_RADAR_MODE_FAR_STATIC      = 0x06,
    UCI_RADAR_MODE_ANT_ISOLATION   = 0x20,
} uci_radar_mode_t;

/* ── RADAR_RX_NTF Data Types ── */
typedef enum {
    UCI_RADAR_DATA_CIR_SAMPLES    = 0x00,
    UCI_RADAR_DATA_PRESENCE       = 0x01,
    UCI_RADAR_DATA_ANT_ISOLATION  = 0x20,
    UCI_RADAR_DATA_LPRF_CALIB     = 0x21,
} uci_radar_data_type_t;

/* ── Main control state machine ── */
typedef enum {
	UCI_WAITING_FOR_USER_COMMAND		= 0x00,
    UCI_INIT_SYSTEM    					= 0x01,
	UCI_INIT_UWB_SUBSYSTEM				= 0x02,
	UCI_INIT_UWB_SUBSYSTEM_FAILED		= 0x03,
	UCI_INIT_UWB_RADAR_SESSION			= 0x04,
	UCI_INIT_UWB_RADAR_FAILED			= 0x05,
	UCI_CONFIG_UWB_RADAR				= 0x06,
	UCI_CONFIG_UWB_RADAR_FAILED			= 0x07,
	UCI_START_UWB_RADAR					= 0x08,
	UCI_START_UWB_RADAR_FAILED			= 0x09,
	UCI_UWB_RADAR_RUNNING				= 0x0A,
	UCI_UWB_RADAR_SESSION_TIMEOUT		= 0x0B,
	UCI_UWB_RADAR_SESSION_STOP			= 0x0C,
	UCI_UWB_RADAR_SESSION_STOP_FAILED   = 0x0D,
	UCI_UWB_RADAR_SESSION_DEINIT		= 0x0E,
	UCI_UWB_RADAR_SESSION_DEINIT_FAILED	= 0x0F,
} uci_main_control_states_t;

/* ── Radar RFRI (Frame Repetition Interval) ── */
typedef struct {
    uint32_t ranging_interval_ms;
    uint16_t slot_duration_rstu;
    uint8_t  slots_per_rr;
} uci_radar_rfri_t;

/* ── Radar RX Gain Control ── */
typedef struct {
    uint8_t agc_mode;       /* 0=AGC, 1=force gain, 2=force AGC re-exec */
    uint8_t gain_rxa;       /* 0-31 */
    uint8_t gain_rxb;       /* 0-31 */
    uint8_t gain_rxc;       /* 0-31 */
} uci_radar_rx_gain_t;

/* ── Radar CIR Start Offset ── */
typedef struct {
    uint16_t rxc_start_index;
    uint16_t rxb_start_index;
    uint16_t rxa_start_index;
} uci_radar_cir_start_offset_t;

/* ── Radar Presence Detection Configuration ── */
typedef struct {
    uint8_t  mode;              /* Bit0:enable Bit1:dist+AoA Bit2:all targets Bit4:trigger val */
    uint8_t  periodic_report;   /* Bit0:raw CIRs, Bits2-1:reporting interval */
    uint8_t  sensitivity_q4_4;  /* Q4.4, default 60 (=3.75) */
    uint8_t  gpio_notify;       /* Bits3-0:GPIO line, Bit7:disable NTF */
    uint16_t distance_min_cm;   /* 50-800, default 50 */
    uint16_t distance_max_cm;   /* 50-800, default 800 */
    uint16_t hold_delay_ms;     /* 400-5000, default 1600 */
    int8_t   angle_min_deg;     /* -90 to +90, default -90 */
    int8_t   angle_max_deg;     /* -90 to +90, default +90 */
} uci_radar_presence_cfg_t;

/* ── Radar LPRF Calibration Configuration ── */
typedef struct {
	uint8_t residue_threshold;
	uint8_t residue_window_length;
}uci_radar_lprf_calibration_cfg_t;

/* ── Radar Session Parameters ── */
typedef struct {
    uci_radar_mode_t mode;
    uint8_t  channel_number;
    uint8_t  preamble_code_index;
    uint8_t  cir_num_samples;
    uint8_t  single_frame_ntf;
    uint8_t  performance;
    uint16_t drift_compensation;
    uint16_t max_measurements;

    bool                        		use_rfri;
    bool 								use_perform_lprf_cal;
    uci_radar_rfri_t            		rfri;
uci_radar_rx_gain_t         			rx_gain;
    uci_radar_cir_start_offset_t 		cir_start_offset;
    uci_radar_presence_cfg_t    		presence_det;
    uci_radar_lprf_calibration_cfg_t 	lprf_cal;

    /* Antenna config (mandatory) */
    uint8_t  ant_rx_rxc_id;
    uint8_t  ant_rx_rxb_id;
    uint8_t  ant_rx_rxa_id;
    uint8_t  ant_tx_id;
} uci_radar_params_t;

/* ── CIR Metadata (parsed from TAP[0]-TAP[7]) ── */
typedef struct {
    uint32_t cir_counter;
    uint32_t rx_timestamp;
    uint32_t tx_timestamp;
    uint8_t  tap_resolution_bits;   /* 00=16-bit, 01=24-bit, 10=32-bit */
    uint8_t  tap_resolution_cm;     /* 00=15cm, 01=7.5cm */
    uint8_t  tx_gain_index;
    uint8_t  rx_gain_index;
    uint8_t  tx_antenna_id;
    uint8_t  rx_antenna_id;
    uint8_t  rx_path;               /* 1=RXC, 2=RXB, 3=RXA */
    uint8_t  radar_mode;
    uint8_t 	inter_pulse_distance_in_ns;
    uint8_t	correlation_bit_shift;
    uint8_t log_2_delta_L;
    uint8_t tx_pulse_shape_id;
    bool     dc_freeze_enabled;
    bool     single_pin;
    uint16_t nsync_by_8;
    uint16_t peak_real_cir;
    uint16_t num_pulses_used;
    uint16_t cir_start_offset;	/* Start offset of CIRs first tap (within 1-727 for modes 1,2,3; within 1-127 for modes 4,5,6) */
    uint16_t cir_start_offset_fractional; /* 0: Start offset set by host, non-zero: Q0.15 value of fractional CIR offset */
} uci_radar_cir_metadata_t;

/* ── Presence Detection Target ── */
#define UCI_RADAR_MAX_DETECTIONS    5
typedef struct {
    uint16_t distance_cm;
    int8_t   angle_deg;
    float    snr_trigger;
    bool     snr_valid;
} uci_radar_presence_target_t;

/* ── Parsed Presence Detection NTF ── */
typedef struct {
    uint32_t session_handle;
    uint8_t  status;
    bool     presence_detected;
    uint8_t  presence_mode;
    uint8_t  num_detections;
    uci_radar_presence_target_t targets[UCI_RADAR_MAX_DETECTIONS];
} uci_radar_presence_ntf_t;

/* ── Antenna Isolation Report ── */
typedef struct {
    uint32_t session_handle;
    uint8_t  status;
    uint8_t  tx_antenna_id;
    uint8_t  rx_antenna_id;
    uint16_t isolation_db;
} uci_radar_ant_isolation_ntf_t;

/* ========================================================================== */
/*  Ranging Types (secondary)                                                 */
/* ========================================================================== */

typedef struct {
    uint8_t  device_type;
    uint8_t  device_role;
    uint8_t  ranging_round_usage;
    uint8_t  multi_node_mode;
    uint8_t  channel_number;
    uint8_t  number_of_controlees;
    uint16_t device_mac_address;
    uint16_t dst_mac_address[8];
    uint16_t slot_duration_rstu;
    uint32_t ranging_duration_ms;
    uint8_t  sts_config;
    uint8_t  aoa_result_req;
    uint8_t  rframe_config;
    uint8_t  preamble_code_index;
    uint8_t  sfd_id;
    uint8_t  slots_per_rr;
    uint8_t  schedule_mode;
} uci_ranging_params_t;

typedef struct {
    uint16_t mac_address;
    uint8_t  status;
    uint8_t  nlos;
    uint16_t distance_cm;
    int16_t  aoa_azimuth_deg_q7;
    int16_t  aoa_elevation_deg_q7;
    uint8_t  aoa_azimuth_fom;
    uint8_t  aoa_elevation_fom;
    int16_t  rssi_dbm;
    int16_t  pdoa_q7;
} uci_ranging_measurement_t;

typedef struct {
    uint32_t sequence_number;
    uint32_t session_handle;
    uint8_t  ranging_measurement_type;
    uint8_t  mac_addressing_mode;
    uint8_t  num_measurements;
    uci_ranging_measurement_t measurements[8];
} uci_session_info_ntf_t;

/* ========================================================================== */
/*  SR250 Antenna Configuration Types                                         */
/* ========================================================================== */

typedef struct {
    uint8_t antenna_id;
    uint8_t rx_port;        /* NXP_RX_PORT_RXB / RXA2 / RXA1 */
    uint8_t gpio_mask_lsb;
    uint8_t gpio_mask_msb;
    uint8_t gpio_state_lsb;
    uint8_t gpio_state_msb;
} uci_rx_antenna_def_t;

typedef struct {
    uint8_t antenna_id;
    uint8_t tx_port;        /* NXP_TX_PORT_TRA1 / TRA2 */
    uint8_t gpio_mask_lsb;
    uint8_t gpio_mask_msb;
    uint8_t gpio_state_lsb;
    uint8_t gpio_state_msb;
} uci_tx_antenna_def_t;

typedef struct {
    uint8_t pair_id;
    uint8_t antenna_rxc;
    uint8_t antenna_rxb;
    uint8_t antenna_rxa;
    uint8_t rfu_lsb;
    uint8_t rfu_msb;
} uci_rx_antenna_pair_t;

/* ========================================================================== */
/*  SR250 Init Configuration                                                  */
/* ========================================================================== */

typedef struct {
    const uci_rx_antenna_def_t  *rx_antennas;
    uint8_t                      num_rx_antennas;
    const uci_tx_antenna_def_t  *tx_antennas;
    uint8_t                      num_tx_antennas;
    const uci_rx_antenna_pair_t *rx_pairs;
    uint8_t                      num_rx_pairs;
    bool     run_chip_calibration;
    uint8_t  channel;
} uci_sr250_init_config_t;

#endif /* UCI_TYPES_H */

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


# PROTOCOL_MODE = "uwb_sw"
PROTOCOL_MODE = "legacy_tlv"

# ---------------------------------------------------------------------------
# Session settings — change these two values each recording.
# ---------------------------------------------------------------------------
RECORDING_DURATION_MS = 50_000   # milliseconds
BAG_NAME              = "test1"  # "" → auto timestamp name

# ---------------------------------------------------------------------------
# Active radar preset — pick one key from RADAR_PRESETS below.
# ---------------------------------------------------------------------------
ACTIVE_PRESET = "medium"         # "medium" or "far"

# ---------------------------------------------------------------------------
# Radar presets — mirrors radar_cfg[] in uwb_sw/Core/Src/main.c.
# Each entry maps directly to SET_CONFIG_FULL fields sent to the STM32.
# ---------------------------------------------------------------------------
RADAR_PRESETS = {
    # ── Preset 0: UCI_RADAR_MODE_MEDIUM_DISTANCE (0x01) ──────────────────
    "medium": {
        "radar_preset_index":        0,
        "radar_mode":                0x01,   # MEDIUM_DISTANCE, up to ~2 m
        "channel_number":            9,
        "preamble_code_index":       26,
        "antennas_config_rx_mode":   0x02,
        "ant_rx_rxc_id":             1,
        "ant_rx_rxb_id":             2,
        "ant_rx_rxa_id":             0,      # 0 = disabled
        "ant_tx_id":                 1,
        "single_frame_ntf":          0,
        "rfri_ranging_interval_ms":  100,
        "rfri_slot_duration_rstu":   12000,
        "rfri_slots_per_rr":         10,
        "cir_num_samples":           128,
        "rx_gain_agc_mode":          0,      # 0 = AGC
        "rx_gain_rxa":               0,
        "rx_gain_rxb":               0,
        "rx_gain_rxc":               0,
        "cir_start_offset_rxc":      0,
        "cir_start_offset_rxb":      0,
        "cir_start_offset_rxa":      0,
        "performance":               0x03,
        "drift_compensation":        0x0CCD,
        "presence_mode":             0x00,
        "presence_periodic_report":  0x04,
        "presence_sensitivity_q4_4": 60,
        "presence_gpio_notify":      0x00,
        "presence_distance_min_cm":  30,
        "presence_distance_max_cm":  200,
        "presence_hold_delay_ms":    1600,
        "presence_angle_min_deg":    -90,
        "presence_angle_max_deg":    90,
    },
    # ── Preset 1: UCI_RADAR_MODE_FAR_DISTANCE (0x03) ─────────────────────
    "far": {
        "radar_preset_index":        1,
        "radar_mode":                0x03,   # FAR_DISTANCE
        "channel_number":            9,
        "preamble_code_index":       26,
        "antennas_config_rx_mode":   0x02,
        "ant_rx_rxc_id":             1,
        "ant_rx_rxb_id":             2,
        "ant_rx_rxa_id":             0,
        "ant_tx_id":                 1,
        "single_frame_ntf":          0,
        "rfri_ranging_interval_ms":  100,
        "rfri_slot_duration_rstu":   12000,
        "rfri_slots_per_rr":         10,
        "cir_num_samples":           128,
        "rx_gain_agc_mode":          0,
        "rx_gain_rxa":               0,
        "rx_gain_rxb":               0,
        "rx_gain_rxc":               0,
        "cir_start_offset_rxc":      0,
        "cir_start_offset_rxb":      0,
        "cir_start_offset_rxa":      0,
        "performance":               0x03,
        "drift_compensation":        0x0CCD,
        "presence_mode":             0x00,
        "presence_periodic_report":  0x04,
        "presence_sensitivity_q4_4": 60,
        "presence_gpio_notify":      0x00,
        "presence_distance_min_cm":  30,
        "presence_distance_max_cm":  200,
        "presence_hold_delay_ms":    1600,
        "presence_angle_min_deg":    -90,
        "presence_angle_max_deg":    90,
    },
}

# ---------------------------------------------------------------------------
# Network addressing
# ---------------------------------------------------------------------------
STM32_IP   = "192.168.1.10"    # board IP (MAC 00:80:E1 = STMicro)
STM32_PORT = 37249              # command port
PC_LISTEN_ACK_PORT    = 20001   # legacy_tlv ACK listener
PC_LISTEN_FRAME_PORT  = 20000   # CIR frame listener

# ---------------------------------------------------------------------------
# Full config sent to uwb_node — common transport fields + selected preset.
# ---------------------------------------------------------------------------
RADAR_CFG = {
    "protocol_mode":       PROTOCOL_MODE,
    "stm32_ip":            STM32_IP,
    "stm32_port":          STM32_PORT,
    "listen_port":         PC_LISTEN_ACK_PORT,
    "auto_start":          True,
    "session_duration_ms": RECORDING_DURATION_MS,
    **RADAR_PRESETS[ACTIVE_PRESET],
}

# Inspector only needs the subset that goes into its summary header
_INSPECTOR_KEYS = (
    "stm32_ip",
    "channel_number", "preamble_code_index", "radar_mode",
    "rfri_ranging_interval_ms", "rfri_slot_duration_rstu", "rfri_slots_per_rr",
    "cir_num_samples", "ant_rx_rxc_id", "ant_rx_rxb_id", "ant_rx_rxa_id",
    "ant_tx_id", "rx_gain_agc_mode", "performance", "drift_compensation",
    "presence_mode",
)
INSPECTOR_CFG = {k: RADAR_CFG[k] for k in _INSPECTOR_KEYS}
INSPECTOR_CFG["summary_path"] = "/tmp/session_summary.txt"
INSPECTOR_CFG["protocol_mode"] = PROTOCOL_MODE


def generate_launch_description():
    return LaunchDescription([

        # ── 0. Timestamp publisher ───────────────────────────────────────────
        # Publishes Unix timestamps for cross-node time alignment.
        Node(
            package="sensors",
            executable="unix_timestamp",
            name="unix_timestamp",
            output="screen",
        ),

        # ── 1. UDP → ROS2 bridge ─────────────────────────────────────────────
        # Listens on port 20000 for RADAR_FRAME UDP packets from the STM32,
        # strips the protocol envelope, and publishes each frame as UwbFrame
        # on /uwb/frame_raw.
        Node(
            package="sensors",
            executable="uwb_udp_frame_publisher",
            name="uwb_udp_frame_publisher",
            output="screen",
            parameters=[{
                "protocol_mode": PROTOCOL_MODE,
                "listen_ip":   "0.0.0.0",
                "listen_port": PC_LISTEN_FRAME_PORT,
                "topic_name":  "/uwb/frame_raw",
            }],
        ),

        # ── 2. CIR inspector / test node ─────────────────────────────────────
        # Subscribes to /uwb/frame_raw, decodes every CIR frame fully, and
        # writes a rolling /tmp/session_summary.txt showing metadata, antenna
        # info, tap statistics, and first tap values per frame.
        Node(
            package="sensors",
            executable="uwb_cir_inspector",
            name="uwb_cir_inspector",
            output="screen",
            parameters=[INSPECTOR_CFG],
        ),

        # ── 3. Frame parser / archiver ───────────────────────────────────────
        # Subscribes to /uwb/frame_raw and saves each CIR frame as a
        # compressed NumPy archive (.npz) in ./uwb_npz/.
        Node(
            package="sensors",
            executable="uwb_frame_parser_node",
            name="uwb_frame_parser_node",
            output="screen",
            parameters=[{
                "topic_name":  "/uwb/frame_raw",
                "output_dir":  "/home/ws/src/uwb_npz",
                "save_every_n": 1,
            }],
        ),

        # ── 4. Rosbag recorder ───────────────────────────────────────────────
        # Waits for the first CIR frame, then records /uwb/frame_raw to an
        # MCAP rosbag for exactly RECORDING_DURATION_MS milliseconds.
        Node(
            package="sensors",
            executable="uwb_rosbag_recorder_node",
            name="uwb_rosbag_recorder_node",
            output="screen",
            parameters=[{
                "topic_name":            "/uwb/frame_raw",
                "output_dir":            "/home/ws/src/uwb_rosbags",
                "bag_prefix":            "uwb_session",
                "bag_name":              BAG_NAME,
                "recording_duration_ms": RECORDING_DURATION_MS,
            }],
        ),

        # ── 5. Control node (delayed so receivers are ready first) ────────────
        # uwb_sw mode: sends CMD_START_SESSION to STM32 port 37249 with the
        # configured preset and duration (= RECORDING_DURATION_MS). The STM32
        # streams RADAR_FRAME packets for that duration, then stops.
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="sensors",
                    executable="uwb_node",
                    name="uwb_node",
                    output="screen",
                    parameters=[RADAR_CFG],
                ),
            ],
        ),

        # ── 5. Synthetic-data test sender (hardware-free pipeline test) ───────
        # Sends synthetic CIR frames to 127.0.0.1:20000 at 10 Hz so the
        # full ROS2 pipeline can be verified without the STM32.
        #
        # DISABLE THIS when the STM32 is connected — it injects synthetic
        # packets that would mix with real radar data on port 20000.
        # To disable: comment out the TimerAction block below.
        #
        # TimerAction(
        #     period=4.0,
        #     actions=[
        #         Node(
        #             package="sensors",
        #             executable="uwb_test_sender",
        #             name="uwb_test_sender",
        #             output="screen",
        #             parameters=[{
        #                 "target_ip":               "127.0.0.1",
        #                 "target_port":             20000,
        #                 "rate_hz":                 10.0,
        #                 "num_samples":             2,
        #                 "taps_per_block":          120,
        #                 "wrap_protocol_envelope":  True,
        #             }],
        #         ),
        #     ],
        # ),

    ])

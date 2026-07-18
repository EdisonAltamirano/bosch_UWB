from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


# ---------------------------------------------------------------------------
# Session settings — change these per recording, same convention as
# sensors/launch/sensors.launch.py.
# ---------------------------------------------------------------------------
RECORDING_DURATION_MS = 10_000   # milliseconds
BAG_NAME              = ""       # "" -> auto timestamp name
SETTING_IDX           = 0        # on-device radar preset index (uwb_sw has no
                                  # host-pushed TLV config — the board selects
                                  # its own preset by this index)

# ---------------------------------------------------------------------------
# Network addressing — uwb_sw uses a single port for everything the board
# sends (ACK/ERROR/SYSTEM_STATE/CIR_REPORT), unlike legacy_tlv's separate
# ack/frame ports. See debug_logs/D0_firmware_inventory.md.
# ---------------------------------------------------------------------------
BOARD_IP           = "192.168.1.10"
BOARD_PORT         = 37249   # NUCLEO_ETH_PORT — PC -> board commands
PC_LISTEN_PORT     = 20000   # NUCLEO_REMOTE_PORT — board -> PC, all packet types


def generate_launch_description():
    return LaunchDescription([

        # ── 0. Timestamp publisher ───────────────────────────────────────────
        Node(
            package="sensors",
            executable="unix_timestamp",
            name="unix_timestamp",
            output="screen",
        ),

        # ── 1. UDP -> ROS2 bridge ────────────────────────────────────────────
        # Owns the one receive socket on PC_LISTEN_PORT. Decodes CMD_* packets,
        # reassembles CIR_REPORT fragments, publishes UwbFrame on
        # /uwb/frame_raw, and republishes ACK/ERROR/SYSTEM_STATE as a status
        # topic for new_uwb_node to consume.
        Node(
            package="new_uwb",
            executable="new_uwb_udp_frame_publisher",
            name="new_uwb_udp_frame_publisher",
            output="screen",
            parameters=[{
                "listen_ip":   "0.0.0.0",
                "listen_port": PC_LISTEN_PORT,
                "topic_name":  "/uwb/frame_raw",
                "status_topic_name": "/uwb/new_uwb_control_status",
            }],
        ),

        # ── 2. Rosbag recorder ───────────────────────────────────────────────
        # Reused unmodified from sensors — it only knows about the topic name,
        # not the protocol that produced it.
        Node(
            package="sensors",
            executable="uwb_rosbag_recorder_node",
            name="uwb_rosbag_recorder_node",
            output="screen",
            parameters=[{
                "topic_name":            "/uwb/frame_raw",
                "output_dir":            "/home/ws/src/uwb_rosbags",
                "bag_prefix":            "new_uwb_session",
                "bag_name":              BAG_NAME,
                "recording_duration_ms": RECORDING_DURATION_MS,
            }],
        ),

        # ── 3. Control node (delayed so the receiver is ready first) ─────────
        # Sends CMD_START_SESSION to the board; does not bind a receive
        # socket (see new_uwb_node.py docstring).
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="new_uwb",
                    executable="new_uwb_node",
                    name="new_uwb_node",
                    output="screen",
                    parameters=[{
                        "board_ip":     BOARD_IP,
                        "board_port":   BOARD_PORT,
                        "setting_idx":  SETTING_IDX,
                        "duration_ms":  RECORDING_DURATION_MS,
                        "auto_start":   True,
                        "status_topic_name": "/uwb/new_uwb_control_status",
                    }],
                ),
            ],
        ),

    ])

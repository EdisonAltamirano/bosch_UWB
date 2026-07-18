#!/usr/bin/env python3
"""ROS2 control node for the uwb_sw (NXP board) CMD_* protocol.

Send-only: this node opens a UDP socket purely to sendto() the board's
command port (37249) and never binds/receives on it. All inbound traffic
(ACK/ERROR/SYSTEM_STATE/CIR_REPORT) is owned by new_uwb_udp_frame_publisher,
which republishes ACK/ERROR/SYSTEM_STATE as a small JSON status topic — see
that node's docstring and NEW_UWB_PLAN_ADDENDUM.md's D5 socket-ownership
note for why control and receive are split this way instead of each node
binding its own socket on the board's single data port.

Unlike legacy_tlv, uwb_sw has no SET_CONFIG_FULL/partial-config/STOP_RADAR
equivalent on the wire today — the board only accepts CMD_START_SESSION
(preset index + duration) and CMD_SYSTEM_COMMAND (currently just remote
reset). Radar configuration itself lives on-device, selected by
`setting_idx`, not pushed from the PC.
"""
from __future__ import annotations

import json
import socket
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from new_uwb.protocol import (
    NUCLEO_ETH_PORT,
    build_start_session_packet,
    build_system_reset_packet,
)


class NewUwbNode(Node):
    def __init__(self):
        super().__init__("new_uwb_node")

        self.declare_parameter("board_ip", "192.168.1.10")
        self.declare_parameter("board_port", NUCLEO_ETH_PORT)
        self.declare_parameter("setting_idx", 0)
        self.declare_parameter("duration_ms", 10_000)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("status_topic_name", "/uwb/new_uwb_control_status")

        self.board_ip = self.get_parameter("board_ip").value
        self.board_port = int(self.get_parameter("board_port").value)
        self.setting_idx = int(self.get_parameter("setting_idx").value)
        self.duration_ms = int(self.get_parameter("duration_ms").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)
        status_topic_name = self.get_parameter("status_topic_name").value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._status_sub = self.create_subscription(
            String, status_topic_name, self._on_status, 10
        )

        self.get_logger().info(
            f"new_uwb_node (CMD_* protocol) ready. Target: "
            f"{self.board_ip}:{self.board_port}, setting_idx={self.setting_idx}, "
            f"duration_ms={self.duration_ms}"
        )

        if self.auto_start:
            self.send_start_session()

    def send_start_session(self) -> None:
        packet = build_start_session_packet(self.setting_idx, self.duration_ms)
        self.sock.sendto(packet, (self.board_ip, self.board_port))
        self.get_logger().info(
            f"Sent CMD_START_SESSION setting_idx={self.setting_idx} "
            f"duration_ms={self.duration_ms} raw={packet.hex(' ')}"
        )

    def send_system_reset(self) -> None:
        packet = build_system_reset_packet()
        self.sock.sendto(packet, (self.board_ip, self.board_port))
        self.get_logger().info(f"Sent CMD_SYSTEM_COMMAND/RESET raw={packet.hex(' ')}")

    def _on_status(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f"Malformed status payload: {msg.data!r}")
            return

        kind = event.get("event")
        if kind == "ack":
            self.get_logger().info(f"ACK recv_cmd_id=0x{event['recv_cmd_id']:02X}")
        elif kind == "error":
            self.get_logger().error(f"ERROR {event['err_name']} (0x{event['err_id']:02X})")
        elif kind == "system_state":
            self.get_logger().info(f"SYSTEM_STATE {event['state_name']}")
        else:
            self.get_logger().warning(f"Unknown status event: {event}")

    def destroy_node(self):
        try:
            self.sock.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node: Optional[NewUwbNode] = None
    try:
        node = NewUwbNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

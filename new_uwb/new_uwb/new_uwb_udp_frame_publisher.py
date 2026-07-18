#!/usr/bin/env python3
"""UDP -> ROS2 bridge for the uwb_sw (NXP board) CMD_* protocol.

Owns the single UDP socket on `listen_port` (board sends ACK/ERROR/
SYSTEM_STATE/CIR_REPORT all to the same port — see
debug_logs/D0_firmware_inventory.md). This is a deliberate single-owner
design: new_uwb_node.py sends commands but does not bind a receive socket,
so there is exactly one process listening on the board's data port at a
time (see NEW_UWB_PLAN_ADDENDUM.md, D5 socket-ownership note).

Publishes:
- UwbFrame on `topic_name` (default /uwb/frame_raw) for each reassembled
  CIR frame — same message type/topic as legacy_tlv, so rosbags recorded
  from either protocol are interchangeable for uwb_processing.
- std_msgs/String (JSON payload) on `status_topic_name` (default
  /uwb/new_uwb_control_status) for ACK/ERROR/SYSTEM_STATE events, so
  new_uwb_node can log/react to them without a second socket on the board's
  data port.
"""
from __future__ import annotations

import json
import socket
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from new_uwb.protocol import (
    CIRReassembler,
    identify_packet,
    parse_ack_packet,
    parse_cir_fragment_packet,
    parse_error_packet,
    parse_system_state_packet,
)

try:
    from sensors.sr250_protocol import MSG_TYPE_RADAR_FRAME, parse_cir_udp_payload
except ImportError:
    from sr250_protocol import MSG_TYPE_RADAR_FRAME, parse_cir_udp_payload  # type: ignore

try:
    from sensors_interfaces.msg import UwbFrame as UWBFrameMsg
except ImportError:
    try:
        from sensors_interfaces.msg import UWBFrame as UWBFrameMsg
    except ImportError:
        from sensors_interfaces.msg import UWB_Frame as UWBFrameMsg  # type: ignore


class NewUwbUdpFramePublisher(Node):
    def __init__(self):
        super().__init__("new_uwb_udp_frame_publisher")

        self.declare_parameter("listen_ip", "0.0.0.0")
        self.declare_parameter("listen_port", 20000)
        self.declare_parameter("topic_name", "/uwb/frame_raw")
        self.declare_parameter("status_topic_name", "/uwb/new_uwb_control_status")

        self.listen_ip = self.get_parameter("listen_ip").value
        self.listen_port = int(self.get_parameter("listen_port").value)
        topic_name = self.get_parameter("topic_name").value
        status_topic_name = self.get_parameter("status_topic_name").value

        self.publisher_ = self.create_publisher(UWBFrameMsg, topic_name, 10)
        self.status_publisher_ = self.create_publisher(String, status_topic_name, 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.sock.bind((self.listen_ip, self.listen_port))
        self.sock.setblocking(False)

        self._running = True
        self._timer = self.create_timer(0.01, self._poll_socket)
        self._reassembler = CIRReassembler()
        self._seq = 0

        self._datagram_count = 0
        self._ack_count = 0
        self._error_count = 0
        self._system_state_count = 0
        self._cir_fragment_count = 0
        self._cir_frame_count = 0
        self._malformed_count = 0
        self._last_datagram_time = 0.0
        self._status_timer = self.create_timer(5.0, self._log_status)

        self.get_logger().info(
            f"new_uwb (CMD_* protocol) listening on {self.listen_ip}:{self.listen_port}, "
            f"publishing frames on {topic_name}, status on {status_topic_name}"
        )

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        if self._seq == 0:
            self._seq = 1
        return self._seq

    def _publish_status(self, event: str, **fields) -> None:
        msg = String()
        msg.data = json.dumps({"event": event, **fields})
        self.status_publisher_.publish(msg)

    def _publish_cir_frame(self, raw_payload: bytes) -> None:
        if len(raw_payload) <= 5:
            self.get_logger().warning(
                f"Reassembled CIR payload too short to carry a radar_data_type "
                f"byte ({len(raw_payload)} bytes) — dropping"
            )
            self._malformed_count += 1
            return

        radar_data_type = raw_payload[5]
        num_samples = 0
        block_size = 0
        bytes_per_tap = 0
        session_handle = 0
        status = 0

        if radar_data_type == 0x00:
            try:
                parsed = parse_cir_udp_payload(raw_payload)
            except Exception as exc:
                self.get_logger().warning(f"Failed to parse reassembled CIR payload: {exc}")
                self._malformed_count += 1
                return
            num_samples = parsed["num_samples"]
            block_size = parsed["block_size"]
            bytes_per_tap = parsed["bytes_per_tap"]
            session_handle = parsed["session_handle"]
            status = parsed["status"]

        msg = UWBFrameMsg()
        msg.stamp = self.get_clock().now().to_msg()
        msg.seq = self._next_seq()
        msg.msg_type = MSG_TYPE_RADAR_FRAME
        msg.radar_data_type = radar_data_type
        msg.session_handle = session_handle
        msg.status = status
        msg.num_samples = num_samples
        msg.block_size = block_size
        msg.bytes_per_tap = bytes_per_tap
        msg.raw_payload = list(raw_payload)

        self.publisher_.publish(msg)
        self._cir_frame_count += 1

    def _poll_socket(self) -> None:
        if not self._running:
            return

        while True:
            try:
                data, addr = self.sock.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError:
                return
            except Exception as exc:
                self.get_logger().error(f"UDP receive error: {exc}")
                return

            self._datagram_count += 1
            self._last_datagram_time = time.monotonic()
            self._handle_datagram(data, addr)

    def _handle_datagram(self, data: bytes, addr) -> None:
        name = identify_packet(data)

        if name == "ACK":
            try:
                ack = parse_ack_packet(data)
            except Exception as exc:
                self.get_logger().warning(f"Failed to parse ACK from {addr[0]}:{addr[1]}: {exc}")
                self._malformed_count += 1
                return
            self._ack_count += 1
            self.get_logger().info(f"ACK recv_cmd_id=0x{ack['recv_cmd_id']:02X}")
            self._publish_status("ack", recv_cmd_id=ack["recv_cmd_id"])
            return

        if name == "ERROR_REPORT":
            try:
                err = parse_error_packet(data)
            except Exception as exc:
                self.get_logger().warning(f"Failed to parse ERROR from {addr[0]}:{addr[1]}: {exc}")
                self._malformed_count += 1
                return
            self._error_count += 1
            self.get_logger().error(f"ERROR err_id=0x{err['err_id']:02X} ({err['err_name']})")
            self._publish_status("error", err_id=err["err_id"], err_name=err["err_name"])
            return

        if name == "SYSTEM_STATE":
            try:
                state = parse_system_state_packet(data)
            except Exception as exc:
                self.get_logger().warning(f"Failed to parse SYSTEM_STATE from {addr[0]}:{addr[1]}: {exc}")
                self._malformed_count += 1
                return
            self._system_state_count += 1
            self.get_logger().info(f"SYSTEM_STATE {state['state_name']}")
            self._publish_status("system_state", state_id=state["state_id"], state_name=state["state_name"])
            return

        if name == "CIR_REPORT":
            try:
                fragment = parse_cir_fragment_packet(data)
            except Exception as exc:
                self.get_logger().warning(f"Failed to parse CIR fragment from {addr[0]}:{addr[1]}: {exc}")
                self._malformed_count += 1
                self._reassembler.reset()
                return
            self._cir_fragment_count += 1
            raw_payload = self._reassembler.feed(fragment)
            if raw_payload is not None:
                self._publish_cir_frame(raw_payload)
            return

        self.get_logger().warning(
            f"Dropping unknown packet from {addr[0]}:{addr[1]} cmd={name} len={len(data)}"
        )
        self._malformed_count += 1

    def _log_status(self) -> None:
        now = time.monotonic()
        age = (now - self._last_datagram_time) if self._last_datagram_time else None
        self.get_logger().info(
            "status: "
            f"datagrams={self._datagram_count} "
            f"ack={self._ack_count} error={self._error_count} "
            f"system_state={self._system_state_count} "
            f"cir_fragments={self._cir_fragment_count} "
            f"cir_frames={self._cir_frame_count} "
            f"malformed={self._malformed_count} "
            f"reassembly_buffer_bytes={len(self._reassembler)} "
            f"last_datagram_age={'n/a' if age is None else f'{age:.2f}s'}"
        )

    def destroy_node(self):
        self._running = False
        try:
            self.sock.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node: Optional[NewUwbUdpFramePublisher] = None
    try:
        node = NewUwbUdpFramePublisher()
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

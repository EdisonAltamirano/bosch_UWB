#!/usr/bin/env python3
"""ROS2 test node that emits synthetic uwb_sw (CMD_*) UDP frames.

Hardware-free pipeline test, mirroring sensors/sensors/uwb_test_sender.py's
role for legacy_tlv: sends the same raw SR250 CIR payload shape (same field
values, so a human can diff the two protocols' framing directly), but wraps
it in CMD_CIR_REPORT fragments the way uwb_sw's nucleo_udp_send() does
instead of the TLV envelope. Lets new_uwb_udp_frame_publisher, the rosbag
recorder, and uwb_processing all be exercised without a flashed board.

DISABLE when the real board is connected — it injects synthetic packets that
would mix with real traffic on the same port.
"""
from __future__ import annotations

import math
import struct

import rclpy
from rclpy.node import Node

from new_uwb.protocol import CMD_CIR_REPORT, MAX_DATA_PER_ETH_PACKET
import socket


class NewUwbTestSender(Node):
    def __init__(self):
        super().__init__("new_uwb_test_sender")

        self.declare_parameter("target_ip", "127.0.0.1")
        self.declare_parameter("target_port", 20000)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("num_samples", 8)
        self.declare_parameter("taps_per_block", 120)

        target_ip = self.get_parameter("target_ip").value
        target_port = int(self.get_parameter("target_port").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.num_samples = int(self.get_parameter("num_samples").value)
        self.taps_per_block = int(self.get_parameter("taps_per_block").value)

        self.target = (target_ip, target_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._frame = 0

        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self._send_frame)

        self.get_logger().info(
            f"new_uwb synthetic CMD_CIR_REPORT sender ready: "
            f"{target_ip}:{target_port} at {rate_hz:.1f} Hz, "
            f"{self.num_samples} blocks x {self.taps_per_block} taps"
        )

    def _build_sample_metadata(self, sample_idx: int) -> bytes:
        metadata = bytearray(32)
        cir_counter = 5 + (self._frame // 2)
        rx_timestamp = (self._frame * 10000) + (sample_idx * 100)
        tx_timestamp = max(0, rx_timestamp - 40)
        rx_path = 1 if (sample_idx % 2 == 0) else 2

        struct.pack_into("<I", metadata, 0, cir_counter)
        struct.pack_into("<I", metadata, 4, rx_timestamp)
        struct.pack_into("<I", metadata, 12, tx_timestamp)
        metadata[16] = rx_path
        metadata[17] = 0x01
        metadata[18] = rx_path
        metadata[19] = 1
        metadata[20] = 10
        struct.pack_into("<H", metadata, 24, 512)
        struct.pack_into("<H", metadata, 26, 16)
        return bytes(metadata)

    def _build_sample_taps(self, sample_idx: int) -> bytes:
        taps = bytearray()
        phase_offset = (self._frame * 0.08) + (sample_idx * 0.35)
        for tap_idx in range(self.taps_per_block):
            angle = (2.0 * math.pi * tap_idx / max(1, self.taps_per_block)) + phase_offset
            amplitude = 1200.0 * math.exp(-tap_idx * 0.03)
            real = int(amplitude * math.cos(angle))
            imag = int(amplitude * math.sin(angle))
            taps.extend(struct.pack("<hh", real, imag))
        return bytes(taps)

    def _build_raw_sr250_payload(self) -> bytes:
        """Same raw NTF layout main.c forwards for both legacy_tlv and
        uwb_sw — see NEW_UWB_PLAN.md Phase 2 and debug_logs/D3_parser_report.md."""
        session_handle = 0x11223344
        status = 0x00
        radar_data_type = 0x00
        taps_per_sample = self.taps_per_block + 8

        payload = bytearray()
        payload.extend(struct.pack("<I", session_handle))
        payload.append(status)
        payload.append(radar_data_type)
        payload.extend(struct.pack("<H", self.num_samples))
        payload.append(taps_per_sample)
        payload.append(0x00)

        for sample_idx in range(self.num_samples):
            payload.extend(self._build_sample_metadata(sample_idx))
            payload.extend(self._build_sample_taps(sample_idx))

        return bytes(payload)

    def _send_frame(self) -> None:
        raw_payload = self._build_raw_sr250_payload()

        # Mirror nucleo_udp_send()'s CMD_CIR_REPORT loop in udp_server.c:
        # split into MAX_DATA_PER_ETH_PACKET-sized fragments, last one flagged.
        offset = 0
        remaining = len(raw_payload)
        packets_sent = 0
        while remaining > 0:
            chunk_len = min(MAX_DATA_PER_ETH_PACKET, remaining)
            last = chunk_len == remaining
            chunk = raw_payload[offset:offset + chunk_len]
            packet = (
                bytes([CMD_CIR_REPORT, 0x01 if last else 0x00])
                + struct.pack("<H", chunk_len)
                + chunk
            )
            self.sock.sendto(packet, self.target)
            offset += chunk_len
            remaining -= chunk_len
            packets_sent += 1

        self._frame += 1
        self.get_logger().info(
            f"Sent synthetic CIR frame #{self._frame} "
            f"({len(raw_payload)} bytes raw, {packets_sent} fragment(s)) "
            f"-> {self.target[0]}:{self.target[1]}"
        )

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = NewUwbTestSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

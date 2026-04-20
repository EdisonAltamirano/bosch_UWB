#!/usr/bin/env python3
"""
ROS2 test node that emits synthetic SR250-compatible UDP frames.

By default it sends `RADAR_FRAME` packets using the custom UWB UDP protocol,
so the full ROS2 pipeline can be exercised without STM32 hardware.
"""

from __future__ import annotations

import math
import socket
import struct

import rclpy
from rclpy.node import Node

try:
    from sensors.sr250_protocol import (
        MSG_TYPE_RADAR_FRAME,
        pack_envelope,
    )
except ImportError:
    from sr250_protocol import (
        MSG_TYPE_RADAR_FRAME,
        pack_envelope,
    )


class UWBTestSender(Node):
    def __init__(self):
        super().__init__("uwb_test_sender")

        self.declare_parameter("target_ip", "127.0.0.1")
        self.declare_parameter("target_port", 20000)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("num_samples", 8)
        self.declare_parameter("taps_per_block", 120)
        self.declare_parameter("wrap_protocol_envelope", True)

        target_ip = self.get_parameter("target_ip").value
        target_port = int(self.get_parameter("target_port").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.num_samples = int(self.get_parameter("num_samples").value)
        self.taps_per_block = int(self.get_parameter("taps_per_block").value)
        self.wrap_protocol_envelope = bool(
            self.get_parameter("wrap_protocol_envelope").value
        )

        self.target = (target_ip, target_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._frame = 0

        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self._send_packet)

        self.get_logger().info(
            f"UWB test sender ready: {target_ip}:{target_port} at {rate_hz:.1f} Hz, "
            f"{self.num_samples} blocks x {self.taps_per_block} taps"
        )

    def _build_sample_metadata(self, sample_idx: int) -> bytes:
        metadata = bytearray(32)
        cir_counter = 5 + (self._frame // 2)
        rx_timestamp = (self._frame * 10000) + (sample_idx * 100)
        tx_timestamp = max(0, rx_timestamp - 40)
        rx_path = 1 if (sample_idx % 2 == 0) else 2
        rx_ant_id = rx_path
        tx_ant_id = 1

        struct.pack_into("<I", metadata, 0, cir_counter)
        struct.pack_into("<I", metadata, 4, rx_timestamp)
        metadata[8] = 0
        metadata[9] = 0
        metadata[10] = 0
        metadata[11] = 0
        struct.pack_into("<I", metadata, 12, tx_timestamp)
        metadata[16] = rx_path
        metadata[17] = 0x01
        metadata[18] = rx_ant_id
        metadata[19] = tx_ant_id
        metadata[20] = 10
        metadata[21] = 0
        metadata[22] = 0
        metadata[23] = 0
        struct.pack_into("<H", metadata, 24, 512)
        struct.pack_into("<H", metadata, 26, 16)
        struct.pack_into("<H", metadata, 28, 0)
        struct.pack_into("<H", metadata, 30, 0)
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

    def _build_udp_packet(self) -> bytes:
        raw_payload = self._build_raw_sr250_payload()

        if not self.wrap_protocol_envelope:
            return raw_payload

        radar_frame_payload = bytearray()
        radar_frame_payload.extend(raw_payload[0:4])
        radar_frame_payload.append(raw_payload[4])
        radar_frame_payload.append(raw_payload[5])
        radar_frame_payload.extend(struct.pack("<H", len(raw_payload)))
        radar_frame_payload.extend(raw_payload)

        seq = self._frame & 0xFFFF
        return pack_envelope(MSG_TYPE_RADAR_FRAME, seq, bytes(radar_frame_payload))

    def _send_packet(self) -> None:
        packet = self._build_udp_packet()
        self.sock.sendto(packet, self.target)
        self._frame += 1

        self.get_logger().info(
            f"Sent synthetic UWB packet #{self._frame} ({len(packet)} bytes) "
            f"-> {self.target[0]}:{self.target[1]}"
        )

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = UWBTestSender()
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

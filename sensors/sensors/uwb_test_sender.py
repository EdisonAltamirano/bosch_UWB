#!/usr/bin/env python3
"""
ROS2 test node that simulates a UWB device sending CIR data packets.

Sends synthetic CIR (Channel Impulse Response) packets to uwb_node's
listen port so you can verify the full pipeline without real hardware.

Each packet contains `cirtaps` complex samples (int16 I + int16 Q),
rotating in phase each frame to simulate a real signal.

Usage:
  Terminal 1: ros2 run sensors uwb_node --ros-args -p udp_ip:=127.0.0.1
  Terminal 2: ros2 run sensors uwb_test_sender
  Terminal 3: ros2 topic echo /uwb/cir_raw
"""
import math
import socket
import struct

import rclpy
from rclpy.node import Node


class UWBTestSender(Node):
    def __init__(self):
        super().__init__('uwb_test_sender')

        # --- Parameters ---
        self.declare_parameter('target_ip', '127.0.0.1')
        self.declare_parameter('target_port', 9999)   # must match uwb_node listen_port
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('cirtaps', 64)

        target_ip = self.get_parameter('target_ip').get_parameter_value().string_value
        target_port = self.get_parameter('target_port').get_parameter_value().integer_value
        rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.cirtaps = self.get_parameter('cirtaps').get_parameter_value().integer_value

        self.target = (target_ip, target_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._frame = 0

        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self._send_packet)

        self.get_logger().info(
            f'UWB test sender ready: {target_ip}:{target_port} '
            f'at {rate_hz:.1f} Hz, {self.cirtaps} taps per packet'
        )

    def _send_packet(self):
        """
        Builds a fake CIR packet: cirtaps complex samples encoded as
        little-endian int16 pairs (I, Q).  Phase rotates each frame.
        """
        samples = []
        for tap in range(self.cirtaps):
            angle = (2 * math.pi * tap / self.cirtaps) + (self._frame * 0.1)
            amplitude = 1000 * math.exp(-tap * 0.05)   # decaying envelope
            I = int(amplitude * math.cos(angle))
            Q = int(amplitude * math.sin(angle))
            samples.append(struct.pack('<hh', I, Q))

        payload = b''.join(samples)
        self.sock.sendto(payload, self.target)
        self._frame += 1

        self.get_logger().info(
            f'Sent test CIR packet #{self._frame} '
            f'({len(payload)} bytes) -> {self.target[0]}:{self.target[1]}'
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


if __name__ == '__main__':
    main()

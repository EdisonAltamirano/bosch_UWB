#!/usr/bin/env python3
"""
ROS2 node for UWB (Ultra-Wideband) CIR data acquisition.

Sends a configuration packet to the UWB device on startup, then listens
for incoming CIR (Channel Impulse Response) packets and publishes them
to /uwb/cir_raw as UInt8MultiArray.
"""
import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension


class UWBNode(Node):
    def __init__(self):
        super().__init__('uwb_node')

        # --- Parameters ---
        self.declare_parameter('udp_ip', '192.168.1.100')
        self.declare_parameter('udp_port', 9998)
        self.declare_parameter('listen_port', 9999)
        self.declare_parameter('cirinterval', 4000)      # Pulse repetition interval in us (1ms..1s)
        self.declare_parameter('cirtaps', 64)            # Number of CIR taps
        self.declare_parameter('frequency', 6500000)     # Tx frequency in KHz (6.4GHz-8GHz)
        self.declare_parameter('ciroffset', 0)           # CIR offset in taps (0..1015)
        self.declare_parameter('mode', 1)                # Bit0=1: correlated data mode
        self.declare_parameter('dccompfiltercoef', 1)    # DC comp filter 0x01..0x63

        self.udp_ip = self.get_parameter('udp_ip').get_parameter_value().string_value
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.listen_port = self.get_parameter('listen_port').get_parameter_value().integer_value
        cirinterval = self.get_parameter('cirinterval').get_parameter_value().integer_value
        cirtaps = self.get_parameter('cirtaps').get_parameter_value().integer_value
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        ciroffset = self.get_parameter('ciroffset').get_parameter_value().integer_value
        mode = self.get_parameter('mode').get_parameter_value().integer_value
        dccompfiltercoef = self.get_parameter('dccompfiltercoef').get_parameter_value().integer_value

        # --- Publisher ---
        self.pub = self.create_publisher(UInt8MultiArray, '/uwb/cir_raw', 10)

        # --- UDP socket: bind first so the device knows where to reply ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Be resilient to quick restarts (TIME_WAIT / lingering sockets)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.bind(('', self.listen_port))
        except OSError as e:
            self.get_logger().error(
                f'Failed to bind UDP listen port {self.listen_port} (already in use?). '
                f'If another launch is running, stop it or change the `listen_port` parameter. '
                f'Original error: {e}'
            )
            raise
        self.sock.settimeout(1.0)

        # Send configuration packet (same format as original uwb.py)
        packetdata = struct.pack(
            '<IIIHBBBBBBBBBHHBBBBBBBBBBBBBBBB',
            cirinterval, cirtaps, frequency, ciroffset,
            0, 0, 0, 0, 0, 0, 0, 0, 0,
            mode, dccompfiltercoef,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        )
        self.sock.sendto(packetdata, (self.udp_ip, self.udp_port))

        self.get_logger().info(
            f'UWB config sent to {self.udp_ip}:{self.udp_port} '
            f'(cirinterval={cirinterval}us, cirtaps={cirtaps}, freq={frequency}KHz)'
        )
        self.get_logger().info(f'Listening for CIR data on port {self.listen_port}')

        # --- Background thread for UDP receive ---
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

        self._packet_count = 0

    def _recv_loop(self):
        while self._running:
            try:
                data, addr = self.sock.recvfrom(4096)
                self._packet_count += 1

                msg = UInt8MultiArray()
                msg.data = list(data)
                dim = MultiArrayDimension()
                dim.label = 'bytes'
                dim.size = len(data)
                dim.stride = len(data)
                msg.layout.dim = [dim]
                self.pub.publish(msg)

                self.get_logger().info(
                    f'[#{self._packet_count}] Received {len(data)} bytes from {addr[0]}:{addr[1]}'
                )
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    self.get_logger().error(f'UDP recv error: {e}')

    def destroy_node(self):
        self._running = False
        self.sock.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = UWBNode()
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

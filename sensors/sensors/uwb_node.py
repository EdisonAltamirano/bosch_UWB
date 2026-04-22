#!/usr/bin/env python3
"""
ROS2 control node for the STM32 <-> SR250 UWB bridge.

This node sends TLV-based radar configuration packets to the STM32 over UDP,
waits for ACK/ERROR responses, and can optionally start the radar session
after a successful full configuration.
"""

from __future__ import annotations

import socket
import struct
import threading
from typing import Optional

import rclpy
from rclpy.node import Node

try:
    from sensors.sr250_protocol import (
        MSG_TYPE_ACK,
        MSG_TYPE_ERROR,
        MSG_TYPE_RADAR_FRAME,
        MSG_TYPE_SET_CONFIG_FULL,
        MSG_TYPE_SET_PARAMS_PARTIAL,
        MSG_TYPE_START_RADAR,
        MSG_TYPE_STOP_RADAR,
        build_partial_radar_config_tlvs,
        build_set_config_full_packet,
        build_set_params_partial_packet,
        build_start_radar_packet,
        build_stop_radar_packet,
        parse_ack_payload,
        parse_error_payload,
        parse_radar_frame_payload,
        unpack_envelope,
    )
except ImportError:
    from sr250_protocol import (
        MSG_TYPE_ACK,
        MSG_TYPE_ERROR,
        MSG_TYPE_RADAR_FRAME,
        MSG_TYPE_SET_CONFIG_FULL,
        MSG_TYPE_SET_PARAMS_PARTIAL,
        MSG_TYPE_START_RADAR,
        MSG_TYPE_STOP_RADAR,
        build_partial_radar_config_tlvs,
        build_set_config_full_packet,
        build_set_params_partial_packet,
        build_start_radar_packet,
        build_stop_radar_packet,
        parse_ack_payload,
        parse_error_payload,
        parse_radar_frame_payload,
        unpack_envelope,
    )


class UWBNode(Node):
    def __init__(self):
        super().__init__("uwb_node")

        self.declare_parameter("stm32_ip", "192.168.1.10")
        self.declare_parameter("stm32_port", 37249)
        self.declare_parameter("listen_ip", "0.0.0.0")
        self.declare_parameter("listen_port", 20001)
        self.declare_parameter("auto_start", True)

        self.declare_parameter("channel_number", 9)
        self.declare_parameter("preamble_code_index", 26)
        self.declare_parameter("antennas_config_rx_mode", 0x02)
        self.declare_parameter("ant_rx_rxc_id", 0x01)
        self.declare_parameter("ant_rx_rxb_id", 0x02)
        self.declare_parameter("ant_rx_rxa_id", 0x00)
        self.declare_parameter("ant_tx_id", 0x01)
        self.declare_parameter("radar_mode", 0x01)
        self.declare_parameter("single_frame_ntf", 0)
        self.declare_parameter("rfri_ranging_interval_ms", 96)
        self.declare_parameter("rfri_slot_duration_rstu", 14400)
        self.declare_parameter("rfri_slots_per_rr", 4)
        self.declare_parameter("cir_num_samples", 128)
        self.declare_parameter("rx_gain_agc_mode", 0)
        self.declare_parameter("rx_gain_rxa", 0)
        self.declare_parameter("rx_gain_rxb", 0)
        self.declare_parameter("rx_gain_rxc", 0)
        self.declare_parameter("cir_start_offset_rxc", 0)
        self.declare_parameter("cir_start_offset_rxb", 0)
        self.declare_parameter("cir_start_offset_rxa", 0)
        self.declare_parameter("performance", 0x03)
        self.declare_parameter("drift_compensation", 0x0CCD)
        self.declare_parameter("presence_mode", 0x00)
        self.declare_parameter("presence_periodic_report", 0x04)
        self.declare_parameter("presence_sensitivity_q4_4", 60)
        self.declare_parameter("presence_gpio_notify", 0x00)
        self.declare_parameter("presence_distance_min_cm", 30)
        self.declare_parameter("presence_distance_max_cm", 200)
        self.declare_parameter("presence_hold_delay_ms", 1600)
        self.declare_parameter("presence_angle_min_deg", -90)
        self.declare_parameter("presence_angle_max_deg", 90)

        self.stm32_ip = self.get_parameter("stm32_ip").value
        self.stm32_port = int(self.get_parameter("stm32_port").value)
        self.listen_ip = self.get_parameter("listen_ip").value
        self.listen_port = int(self.get_parameter("listen_port").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.listen_ip, self.listen_port))
        self.sock.settimeout(1.0)

        self._running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

        self._seq = 1
        self._config_applied = False
        self._radar_running = False
        self._pending_auto_start = self.auto_start

        self.send_full_config(self._collect_config())

        self.get_logger().info(
            f"UWB control ready. Sending commands to {self.stm32_ip}:{self.stm32_port} "
            f"from local port {self.listen_port}"
        )

    def _collect_config(self):
        return {
            "channel_number": int(self.get_parameter("channel_number").value),
            "preamble_code_index": int(self.get_parameter("preamble_code_index").value),
            "antennas_config_rx_mode": int(self.get_parameter("antennas_config_rx_mode").value),
            "ant_rx_rxc_id": int(self.get_parameter("ant_rx_rxc_id").value),
            "ant_rx_rxb_id": int(self.get_parameter("ant_rx_rxb_id").value),
            "ant_rx_rxa_id": int(self.get_parameter("ant_rx_rxa_id").value),
            "ant_tx_id": int(self.get_parameter("ant_tx_id").value),
            "radar_mode": int(self.get_parameter("radar_mode").value),
            "single_frame_ntf": int(self.get_parameter("single_frame_ntf").value),
            "rfri_ranging_interval_ms": int(self.get_parameter("rfri_ranging_interval_ms").value),
            "rfri_slot_duration_rstu": int(self.get_parameter("rfri_slot_duration_rstu").value),
            "rfri_slots_per_rr": int(self.get_parameter("rfri_slots_per_rr").value),
            "cir_num_samples": int(self.get_parameter("cir_num_samples").value),
            "rx_gain_agc_mode": int(self.get_parameter("rx_gain_agc_mode").value),
            "rx_gain_rxa": int(self.get_parameter("rx_gain_rxa").value),
            "rx_gain_rxb": int(self.get_parameter("rx_gain_rxb").value),
            "rx_gain_rxc": int(self.get_parameter("rx_gain_rxc").value),
            "cir_start_offset_rxc": int(self.get_parameter("cir_start_offset_rxc").value),
            "cir_start_offset_rxb": int(self.get_parameter("cir_start_offset_rxb").value),
            "cir_start_offset_rxa": int(self.get_parameter("cir_start_offset_rxa").value),
            "performance": int(self.get_parameter("performance").value),
            "drift_compensation": int(self.get_parameter("drift_compensation").value),
            "presence_mode": int(self.get_parameter("presence_mode").value),
            "presence_periodic_report": int(self.get_parameter("presence_periodic_report").value),
            "presence_sensitivity_q4_4": int(self.get_parameter("presence_sensitivity_q4_4").value),
            "presence_gpio_notify": int(self.get_parameter("presence_gpio_notify").value),
            "presence_distance_min_cm": int(self.get_parameter("presence_distance_min_cm").value),
            "presence_distance_max_cm": int(self.get_parameter("presence_distance_max_cm").value),
            "presence_hold_delay_ms": int(self.get_parameter("presence_hold_delay_ms").value),
            "presence_angle_min_deg": int(self.get_parameter("presence_angle_min_deg").value),
            "presence_angle_max_deg": int(self.get_parameter("presence_angle_max_deg").value),
        }

    def _next_seq(self) -> int:
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFFFF
        if self._seq == 0:
            self._seq = 1
        return seq

    def _send_packet(self, packet: bytes, label: str) -> None:
        self.sock.sendto(packet, (self.stm32_ip, self.stm32_port))
        msg_type = packet[3]
        seq = struct.unpack_from("<H", packet, 4)[0]
        self.get_logger().info(
            f"Sent {label} msg_type=0x{msg_type:02X} seq={seq} len={len(packet)}"
        )

    def send_full_config(self, config: dict) -> None:
        self._config_applied = False
        self._pending_auto_start = self.auto_start
        self._send_packet(
            build_set_config_full_packet(self._next_seq(), config),
            "SET_CONFIG_FULL",
        )

    def send_partial_update(self, tlv_payload: bytes) -> None:
        self._send_packet(
            build_set_params_partial_packet(self._next_seq(), tlv_payload),
            "SET_PARAMS_PARTIAL",
        )

    def send_partial_config(self, partial_config: dict) -> None:
        self.send_partial_update(build_partial_radar_config_tlvs(partial_config))

    def send_start_radar(self) -> None:
        self._send_packet(build_start_radar_packet(self._next_seq()), "START_RADAR")

    def send_stop_radar(self) -> None:
        self._send_packet(build_stop_radar_packet(self._next_seq()), "STOP_RADAR")

    def _recv_loop(self) -> None:
        while self._running:
            try:
                data, addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as exc:
                if self._running:
                    self.get_logger().error(f"UDP receive error: {exc}")
                continue

            try:
                env = unpack_envelope(data)
            except Exception as exc:
                self.get_logger().warning(
                    f"Ignoring non-protocol UDP packet from {addr[0]}:{addr[1]}: {exc}"
                )
                continue

            if env["msg_type"] == MSG_TYPE_ACK:
                ack = parse_ack_payload(env["payload"])
                acked_type = ack["acked_msg_type"]
                if acked_type == MSG_TYPE_SET_CONFIG_FULL:
                    self._config_applied = True
                    if self._pending_auto_start:
                        self._pending_auto_start = False
                        self.send_start_radar()
                elif acked_type == MSG_TYPE_SET_PARAMS_PARTIAL:
                    self._config_applied = True
                elif acked_type == MSG_TYPE_START_RADAR:
                    self._radar_running = True
                elif acked_type == MSG_TYPE_STOP_RADAR:
                    self._radar_running = False
                self.get_logger().info(
                    f"ACK seq={ack['seq']} acked_msg_type=0x{acked_type:02X}"
                )
            elif env["msg_type"] == MSG_TYPE_ERROR:
                err = parse_error_payload(env["payload"])
                failed_type = err["failed_msg_type"]
                if failed_type == MSG_TYPE_SET_CONFIG_FULL:
                    self._config_applied = False
                    self._pending_auto_start = False
                elif failed_type == MSG_TYPE_START_RADAR:
                    self._radar_running = False
                self.get_logger().error(
                    f"ERROR seq={err['seq']} failed_msg_type=0x{failed_type:02X} "
                    f"error_code=0x{err['error_code']:04X}"
                )
            elif env["msg_type"] == MSG_TYPE_RADAR_FRAME:
                frame = parse_radar_frame_payload(env["payload"])
                self.get_logger().info(
                    f"RADAR_FRAME seq={env['seq']} radar_type=0x{frame['radar_data_type']:02X} "
                    f"raw_len={frame['raw_payload_len']}"
                )
            else:
                self.get_logger().warning(
                    f"Received unknown msg_type=0x{env['msg_type']:02X} seq={env['seq']}"
                )

    def destroy_node(self):
        if self._running and self._radar_running:
            try:
                self.send_stop_radar()
            except Exception:
                pass
        self._running = False
        try:
            self.sock.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node: Optional[UWBNode] = None
    try:
        node = UWBNode()
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

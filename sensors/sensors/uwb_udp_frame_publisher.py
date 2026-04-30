#!/usr/bin/env python3
from __future__ import annotations

import socket
from typing import Optional

import rclpy
from rclpy.node import Node

try:
    from sensors.sr250_protocol import (
        CMD_CIR_REPORT,
        CMD_ERROR_REPORT,
        CMD_UDP_ETH_ACK,
        MSG_TYPE_RADAR_FRAME,
        parse_cir_udp_payload,
        parse_radar_frame_payload,
        parse_uwb_sw_ack_packet,
        parse_uwb_sw_cir_fragment_packet,
        parse_uwb_sw_error_packet,
        unpack_envelope,
    )
except ImportError:
    from sr250_protocol import (
        CMD_CIR_REPORT,
        CMD_ERROR_REPORT,
        CMD_UDP_ETH_ACK,
        MSG_TYPE_RADAR_FRAME,
        parse_cir_udp_payload,
        parse_radar_frame_payload,
        parse_uwb_sw_ack_packet,
        parse_uwb_sw_cir_fragment_packet,
        parse_uwb_sw_error_packet,
        unpack_envelope,
    )

try:
    from sensors_interfaces.msg import UwbFrame as UWBFrameMsg
except ImportError:
    try:
        from sensors_interfaces.msg import UWBFrame as UWBFrameMsg
    except ImportError:
        from sensors_interfaces.msg import UWB_Frame as UWBFrameMsg  # type: ignore


class UWBUDPFramePublisher(Node):
    def __init__(self):
        super().__init__("uwb_udp_frame_publisher")

        self.declare_parameter("protocol_mode", "legacy_tlv")
        self.declare_parameter("listen_ip", "0.0.0.0")
        self.declare_parameter("listen_port", 20000)
        self.declare_parameter("topic_name", "/uwb/frame_raw")

        self.protocol_mode = str(self.get_parameter("protocol_mode").value)
        if self.protocol_mode not in {"legacy_tlv", "uwb_sw"}:
            raise ValueError(
                f"Unsupported protocol_mode={self.protocol_mode!r}; expected 'legacy_tlv' or 'uwb_sw'"
            )

        self.listen_ip = self.get_parameter("listen_ip").value
        self.listen_port = int(self.get_parameter("listen_port").value)
        topic_name = self.get_parameter("topic_name").value

        self.publisher_ = self.create_publisher(UWBFrameMsg, topic_name, 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.listen_ip, self.listen_port))
        self.sock.settimeout(1.0)

        self._running = True
        self._timer = self.create_timer(0.01, self._poll_socket)
        self._packet_count = 0
        self._reassembled_seq = 0
        self._fragment_buffer = bytearray()

        self.get_logger().info(
            f"Listening for {self.protocol_mode} UWB packets on "
            f"{self.listen_ip}:{self.listen_port}"
        )

    def _next_seq(self) -> int:
        self._reassembled_seq = (self._reassembled_seq + 1) & 0xFFFF
        if self._reassembled_seq == 0:
            self._reassembled_seq = 1
        return self._reassembled_seq

    def _publish_raw_sr250_payload(
        self,
        raw_payload: bytes,
        seq: int,
        msg_type: int,
        radar_data_type: int,
        session_handle: int,
        status: int,
    ) -> None:
        num_samples = 0
        block_size = 0
        bytes_per_tap = 0

        if radar_data_type == 0x00:
            parsed = parse_cir_udp_payload(raw_payload)
            num_samples = parsed["num_samples"]
            block_size = parsed["block_size"]
            bytes_per_tap = parsed["bytes_per_tap"]

        msg = UWBFrameMsg()
        msg.stamp = self.get_clock().now().to_msg()
        msg.seq = seq
        msg.msg_type = msg_type
        msg.radar_data_type = radar_data_type
        msg.session_handle = session_handle
        msg.status = status
        msg.num_samples = num_samples
        msg.block_size = block_size
        msg.bytes_per_tap = bytes_per_tap
        msg.raw_payload = list(raw_payload)

        self.publisher_.publish(msg)
        self._packet_count += 1
        self.get_logger().info(
            f"Published UWB frame #{self._packet_count} seq={seq} "
            f"type=0x{radar_data_type:02X} num_samples={num_samples}"
        )

    def _poll_socket(self) -> None:
        if not self._running:
            return

        try:
            data, addr = self.sock.recvfrom(65535)
        except socket.timeout:
            return
        except OSError:
            return
        except Exception as exc:
            self.get_logger().error(f"UDP receive error: {exc}")
            return

        if self.protocol_mode == "uwb_sw":
            self._handle_uwb_sw_packet(data, addr)
            return

        try:
            env = unpack_envelope(data)
        except Exception:
            try:
                parsed = parse_cir_udp_payload(data)
            except Exception as exc:
                self.get_logger().warning(
                    f"Dropping invalid UDP payload from {addr[0]}:{addr[1]}: {exc}"
                )
                return

            self._publish_raw_sr250_payload(
                raw_payload=data,
                seq=0,
                msg_type=MSG_TYPE_RADAR_FRAME,
                radar_data_type=parsed["radar_data_type"],
                session_handle=parsed["session_handle"],
                status=parsed["status"],
            )
            return

        if env["msg_type"] != MSG_TYPE_RADAR_FRAME:
            return

        try:
            frame = parse_radar_frame_payload(env["payload"])
            self._publish_raw_sr250_payload(
                raw_payload=frame["raw_payload"],
                seq=env["seq"],
                msg_type=env["msg_type"],
                radar_data_type=frame["radar_data_type"],
                session_handle=frame["session_handle"],
                status=frame["status"],
            )
        except Exception as exc:
            self.get_logger().warning(f"Failed to parse RADAR_FRAME payload: {exc}")

    def _handle_uwb_sw_packet(self, data: bytes, addr) -> None:
        if not data:
            return

        cmd_id = data[0]

        if cmd_id == CMD_UDP_ETH_ACK:
            try:
                ack = parse_uwb_sw_ack_packet(data)
            except Exception as exc:
                self.get_logger().warning(f"Failed to parse uwb_sw ACK packet: {exc}")
                return

            self.get_logger().info(
                f"uwb_sw ACK from {addr[0]}:{addr[1]} recv_cmd_id=0x{ack['recv_cmd_id']:02X}"
            )
            return

        if cmd_id == CMD_ERROR_REPORT:
            try:
                err = parse_uwb_sw_error_packet(data)
            except Exception as exc:
                self.get_logger().warning(f"Failed to parse uwb_sw ERROR packet: {exc}")
                return

            self.get_logger().error(
                f"uwb_sw ERROR from {addr[0]}:{addr[1]} err_id=0x{err['err_id']:02X} "
                f"({err['err_name']})"
            )
            return

        if cmd_id != CMD_CIR_REPORT:
            self.get_logger().warning(
                f"Dropping unknown uwb_sw packet from {addr[0]}:{addr[1]} cmd_id=0x{cmd_id:02X}"
            )
            return

        try:
            fragment = parse_uwb_sw_cir_fragment_packet(data)
        except Exception as exc:
            self.get_logger().warning(f"Failed to parse uwb_sw CIR fragment: {exc}")
            self._fragment_buffer.clear()
            return

        self._fragment_buffer.extend(fragment["data"])

        if not fragment["last_fragment"]:
            return

        raw_payload = bytes(self._fragment_buffer)
        self._fragment_buffer.clear()

        try:
            parsed = parse_cir_udp_payload(raw_payload)
        except Exception as exc:
            self.get_logger().warning(f"Failed to parse reassembled uwb_sw CIR payload: {exc}")
            return

        self._publish_raw_sr250_payload(
            raw_payload=raw_payload,
            seq=self._next_seq(),
            msg_type=MSG_TYPE_RADAR_FRAME,
            radar_data_type=parsed["radar_data_type"],
            session_handle=parsed["session_handle"],
            status=parsed["status"],
        )

    def destroy_node(self):
        self._running = False
        try:
            self.sock.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node: Optional[UWBUDPFramePublisher] = None
    try:
        node = UWBUDPFramePublisher()
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

#!/usr/bin/env python3
from __future__ import annotations

import socket
import time
from typing import Optional

import rclpy
from rclpy.node import Node

try:
    from sensors.sr250_protocol import (
        CMD_CIR_REPORT,
        CMD_ERROR_REPORT,
        CMD_UDP_ETH_ACK,
        MSG_TYPE_RADAR_FRAME,
        MSG_TYPE_RADAR_FRAME_CHUNK,
        parse_cir_udp_payload,
        parse_radar_frame_chunk_payload,
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
        MSG_TYPE_RADAR_FRAME_CHUNK,
        parse_cir_udp_payload,
        parse_radar_frame_chunk_payload,
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
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.sock.bind((self.listen_ip, self.listen_port))
        self.sock.setblocking(False)

        self._running = True
        self._timer = self.create_timer(0.01, self._poll_socket)
        self._packet_count = 0
        self._datagram_count = 0
        self._chunk_datagram_count = 0
        self._chunk_frame_complete_count = 0
        self._last_datagram_time = time.monotonic()
        self._last_publish_time = 0.0
        self._status_prev_datagrams = 0
        self._status_prev_frames = 0
        self._reassembled_seq = 0
        self._fragment_buffer = bytearray()
        # frame_id -> {"total": N, "chunks": {idx: bytes}} for RADAR_FRAME_CHUNK reassembly
        self._chunk_buffers: dict = {}
        # self._status_timer = self.create_timer(1.0, self._log_status)

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
        self._last_publish_time = time.monotonic()

    def _log_status(self) -> None:
        now = time.monotonic()
        datagram_delta = self._datagram_count - self._status_prev_datagrams
        frame_delta = self._packet_count - self._status_prev_frames
        self._status_prev_datagrams = self._datagram_count
        self._status_prev_frames = self._packet_count

        self.get_logger().info(
            "UDP status: "
            f"datagrams={self._datagram_count} (+{datagram_delta}/s) "
            f"published={self._packet_count} (+{frame_delta}/s) "
            f"chunk_datagrams={self._chunk_datagram_count} "
            f"chunk_frames={self._chunk_frame_complete_count} "
            f"incomplete_frames={len(self._chunk_buffers)} "
            f"last_datagram_age={now - self._last_datagram_time:.2f}s "
            f"last_publish_age={(now - self._last_publish_time) if self._last_publish_time else 0.0:.2f}s"
        )

    def _poll_socket(self) -> None:
        if not self._running:
            return

        # Drain all queued datagrams each timer tick. Chunked CIR frames can
        # arrive faster than the ROS timer frequency, so reading only one
        # packet per callback drops chunks and breaks frame reassembly.
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

            if self.protocol_mode == "uwb_sw":
                self._handle_uwb_sw_packet(data, addr)
                continue

            try:
                env = unpack_envelope(data)
            except Exception as exc:
                try:
                    parsed = parse_cir_udp_payload(data)
                except Exception:
                    self.get_logger().warning(
                        f"Dropping invalid UDP payload from {addr[0]}:{addr[1]}: {exc}"
                    )
                    continue

                self._publish_raw_sr250_payload(
                    raw_payload=data,
                    seq=0,
                    msg_type=MSG_TYPE_RADAR_FRAME,
                    radar_data_type=parsed["radar_data_type"],
                    session_handle=parsed["session_handle"],
                    status=parsed["status"],
                )
                continue

            if env["msg_type"] == MSG_TYPE_RADAR_FRAME_CHUNK:
                self._handle_radar_frame_chunk(env)
                continue

            if env["msg_type"] != MSG_TYPE_RADAR_FRAME:
                continue

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

    def _handle_radar_frame_chunk(self, env: dict) -> None:
        try:
            chunk = parse_radar_frame_chunk_payload(env["payload"])
        except Exception as exc:
            self.get_logger().warning(f"Failed to parse RADAR_FRAME_CHUNK: {exc}")
            return

        frame_id = chunk["frame_id"]
        chunk_idx = chunk["chunk_idx"]
        total_chunks = chunk["total_chunks"]
        self._chunk_datagram_count += 1

        if frame_id not in self._chunk_buffers:
            # Evict oldest incomplete frame if buffer is getting large (leak guard)
            if len(self._chunk_buffers) >= 16:
                oldest_id = next(iter(self._chunk_buffers))
                self.get_logger().warning(
                    f"Evicting incomplete chunk frame_id={oldest_id}"
                )
                del self._chunk_buffers[oldest_id]
            self._chunk_buffers[frame_id] = {"total": total_chunks, "chunks": {}}

        buf = self._chunk_buffers[frame_id]
        buf["chunks"][chunk_idx] = bytes(chunk["chunk_data"])

        if len(buf["chunks"]) < buf["total"]:
            return  # still waiting for remaining chunks

        # All chunks received — assemble in index order and publish
        del self._chunk_buffers[frame_id]
        self._chunk_frame_complete_count += 1
        reassembled = b"".join(buf["chunks"][i] for i in range(buf["total"]))
        try:
            frame = parse_radar_frame_payload(reassembled)
            self._publish_raw_sr250_payload(
                raw_payload=frame["raw_payload"],
                seq=env["seq"],
                msg_type=MSG_TYPE_RADAR_FRAME,
                radar_data_type=frame["radar_data_type"],
                session_handle=frame["session_handle"],
                status=frame["status"],
            )
        except Exception as exc:
            self.get_logger().warning(f"Failed to parse reassembled RADAR_FRAME: {exc}")

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

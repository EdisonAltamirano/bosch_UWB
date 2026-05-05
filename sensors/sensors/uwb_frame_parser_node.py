#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node

try:
    from sensors.sr250_protocol import parse_cir_udp_payload
except ImportError:
    from sr250_protocol import parse_cir_udp_payload

try:
    from sensors_interfaces.msg import UwbFrame as UWBFrameMsg
except ImportError:
    try:
        from sensors_interfaces.msg import UWBFrame as UWBFrameMsg
    except ImportError:
        from sensors_interfaces.msg import UWB_Frame as UWBFrameMsg  # type: ignore


class UWBFrameParserNode(Node):
    def __init__(self):
        super().__init__("uwb_frame_parser_node")

        self.declare_parameter("topic_name", "/uwb/frame_raw")
        self.declare_parameter("output_dir", "uwb_npz")
        self.declare_parameter("save_every_n", 1)

        topic_name = self.get_parameter("topic_name").value
        self.output_dir = Path(self.get_parameter("output_dir").value)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.save_every_n = int(self.get_parameter("save_every_n").value)

        self.subscription = self.create_subscription(
            UWBFrameMsg,
            topic_name,
            self._on_frame,
            10,
        )

        self._frame_count = 0
        self.get_logger().info(
            f"Subscribed to {topic_name}. Saving parsed frames to {self.output_dir}"
        )

    def _on_frame(self, msg: UWBFrameMsg) -> None:
        self._frame_count += 1
        if self._frame_count % self.save_every_n != 0:
            return

        if int(msg.radar_data_type) != 0x00:
            self.get_logger().info(
                f"Skipping non-CIR frame #{self._frame_count} radar_data_type=0x{int(msg.radar_data_type):02X}"
            )
            return

        raw_payload = bytes(msg.raw_payload)

        try:
            parsed = parse_cir_udp_payload(raw_payload)
        except Exception as exc:
            self.get_logger().warning(f"Skipping frame {self._frame_count}: {exc}")
            return

        num_samples = parsed["num_samples"]
        taps_per_block = parsed["actual_cir_taps_per_block"]
        if taps_per_block is None:
            taps_per_block = len(parsed["samples"][0]["taps"]) if parsed["samples"] else 0

        taps_real = np.zeros((num_samples, taps_per_block), dtype=np.int16)
        taps_imag = np.zeros((num_samples, taps_per_block), dtype=np.int16)
        cir_counters = np.zeros((num_samples,), dtype=np.uint32)
        rx_paths = np.zeros((num_samples,), dtype=np.uint8)
        rx_antenna_ids = np.zeros((num_samples,), dtype=np.uint8)
        tx_antenna_ids = np.zeros((num_samples,), dtype=np.uint8)
        timestamps_rx = np.zeros((num_samples,), dtype=np.uint32)
        timestamps_tx = np.zeros((num_samples,), dtype=np.uint32)
        cir_start_offsets = np.zeros((num_samples,), dtype=np.uint16)

        for sample_idx, sample in enumerate(parsed["samples"]):
            meta = sample["metadata"]
            cir_counters[sample_idx] = meta["cir_counter"]
            rx_paths[sample_idx] = meta["rx_path"]
            rx_antenna_ids[sample_idx] = meta["rx_antenna_id"]
            tx_antenna_ids[sample_idx] = meta["tx_antenna_id"]
            timestamps_rx[sample_idx] = meta["rx_timestamp"]
            timestamps_tx[sample_idx] = meta["tx_timestamp"]
            cir_start_offsets[sample_idx] = meta["cir_start_offset"]

            for tap_idx, (real_part, imag_part) in enumerate(sample["taps"]):
                taps_real[sample_idx, tap_idx] = real_part
                taps_imag[sample_idx, tap_idx] = imag_part

        filename = (
            f"uwb_frame_seq{int(msg.seq):05d}_"
            f"{msg.stamp.sec}_{msg.stamp.nanosec}.npz"
        )
        output_path = self.output_dir / filename

        # np.savez_compressed(
        #     output_path,
        #     raw_payload=np.frombuffer(raw_payload, dtype=np.uint8),
        #     session_handle=np.uint32(parsed["session_handle"]),
        #     status=np.uint8(parsed["status"]),
        #     radar_data_type=np.uint8(parsed["radar_data_type"]),
        #     num_samples=np.uint16(parsed["num_samples"]),
        #     block_size=np.uint16(parsed["block_size"]),
        #     bytes_per_tap=np.uint8(parsed["bytes_per_tap"]),
        #     cir_counters=cir_counters,
        #     rx_paths=rx_paths,
        #     rx_antenna_ids=rx_antenna_ids,
        #     tx_antenna_ids=tx_antenna_ids,
        #     timestamps_rx=timestamps_rx,
        #     timestamps_tx=timestamps_tx,
        #     cir_start_offsets=cir_start_offsets,
        #     taps_real=taps_real,
        #     taps_imag=taps_imag,
        # )

        # self.get_logger().info(
        #     f"Saved parsed frame #{self._frame_count} to {output_path.name} "
        #     f"with shape {taps_real.shape}"
        # )


def main():
    rclpy.init()
    node: Optional[UWBFrameParserNode] = None
    try:
        node = UWBFrameParserNode()
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

#!/usr/bin/env python3
"""
UWB rosbag recorder node.

Subscribes to /uwb/frame_raw (or a configured topic), waits for the first CIR
frame (confirmation that the STM32 is streaming), then records all incoming
messages to an MCAP rosbag for exactly `recording_duration_ms` milliseconds.

The session duration sent to the STM32 via uwb_node is set to the same value
in sensors.launch.py so the chip stops streaming when the recording ends.
"""

from __future__ import annotations

import os
import shutil
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
import rosbag2_py

try:
    from sensors_interfaces.msg import UwbFrame as UWBFrameMsg
except ImportError:
    try:
        from sensors_interfaces.msg import UWBFrame as UWBFrameMsg
    except ImportError:
        from sensors_interfaces.msg import UWB_Frame as UWBFrameMsg  # type: ignore


class UWBRosbagRecorderNode(Node):
    def __init__(self):
        super().__init__("uwb_rosbag_recorder_node")

        self.declare_parameter("topic_name", "/uwb/frame_raw")
        self.declare_parameter("output_dir", "/home/ws/src/uwb_rosbags")
        self.declare_parameter("bag_prefix", "uwb_session")
        self.declare_parameter("bag_name", "")
        self.declare_parameter("recording_duration_ms", 10_000)

        self._topic_name = self.get_parameter("topic_name").value
        self._output_dir = Path(self.get_parameter("output_dir").value)
        self._bag_prefix = self.get_parameter("bag_prefix").value
        self._bag_name = self.get_parameter("bag_name").value
        self._duration_ms = int(self.get_parameter("recording_duration_ms").value)

        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._writer: Optional[rosbag2_py.SequentialWriter] = None
        self._recording = False
        self._stop_timer = None
        self._progress_timer = None
        self._frame_count = 0
        self._bag_path: Optional[Path] = None
        self._recording_start_ns: Optional[int] = None

        self._sub = self.create_subscription(
            UWBFrameMsg,
            self._topic_name,
            self._on_frame,
            50,
        )

        self.get_logger().info(
            f"Waiting for first CIR frame on {self._topic_name} "
            f"(will record {self._duration_ms} ms to MCAP)"
        )

    # ------------------------------------------------------------------

    def _start_recording(self) -> None:
        if self._bag_name:
            self._bag_path = self._output_dir / self._bag_name
            if self._bag_path.exists():
                shutil.rmtree(self._bag_path)
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._bag_path = self._output_dir / f"{self._bag_prefix}_{timestamp}"

        storage_options = rosbag2_py.StorageOptions(
            uri=str(self._bag_path),
            storage_id="mcap",
        )
        converter_options = rosbag2_py.ConverterOptions("", "")
        self._writer = rosbag2_py.SequentialWriter()
        try:
            self._writer.open(storage_options, converter_options)
        except Exception as e:
            self.get_logger().error(f"Failed to open rosbag: {e}")
            self._writer = None
            return

        topic_info = rosbag2_py.TopicMetadata(
            name=self._topic_name,
            type="sensors_interfaces/msg/UwbFrame",
            serialization_format="cdr",
        )
        self._writer.create_topic(topic_info)
        self._recording = True
        self._recording_start_ns = self.get_clock().now().nanoseconds

        self._stop_timer = self.create_timer(
            self._duration_ms / 1000.0,
            self._stop_recording,
        )
        self._progress_timer = self.create_timer(1.0, self._log_progress)

        self.get_logger().info(
            f"Recording started → {self._bag_path}  "
            f"(duration={self._duration_ms} ms)"
        )

    def _log_progress(self) -> None:
        elapsed_ms = (self.get_clock().now().nanoseconds - self._recording_start_ns) / 1e6
        remaining_ms = max(0.0, self._duration_ms - elapsed_ms)
        self.get_logger().info(
            f"Recording: {elapsed_ms / 1000:.1f}s elapsed, "
            f"{remaining_ms / 1000:.1f}s remaining, "
            f"{self._frame_count} frames"
        )

    def _stop_recording(self) -> None:
        if self._stop_timer is not None:
            self._stop_timer.cancel()
            self._stop_timer = None

        if self._progress_timer is not None:
            self._progress_timer.cancel()
            self._progress_timer = None

        self._recording = False
        if self._writer is not None:
            del self._writer
            self._writer = None

        self.get_logger().info(
            f"Recording finished. {self._frame_count} frames saved → {self._bag_path}"
        )

    # ------------------------------------------------------------------

    def _on_frame(self, msg: UWBFrameMsg) -> None:
        if not self._recording:
            if self._writer is None:
                self._start_recording()
            else:
                return

        if self._writer is None:
            return

        timestamp_ns = self.get_clock().now().nanoseconds
        self._writer.write(
            self._topic_name,
            serialize_message(msg),
            timestamp_ns,
        )
        self._frame_count += 1

    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        if self._recording:
            self._stop_recording()
        super().destroy_node()


def main():
    rclpy.init()
    node: Optional[UWBRosbagRecorderNode] = None
    try:
        node = UWBRosbagRecorderNode()
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

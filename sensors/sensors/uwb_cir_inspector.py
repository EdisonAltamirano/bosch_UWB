#!/usr/bin/env python3
"""
UWB CIR Inspector

Test node that subscribes to /uwb/frame_raw, fully decodes every CIR frame
arriving from the STM32, and writes a rolling session_summary.txt that contains:

  (1) A one-time header describing the radar config that was sent from the PC.
  (2) One decoded block per received frame: session handle, antenna metadata,
      CIR tap statistics, and the first four tap values.

Usage
-----
  ros2 run sensors uwb_cir_inspector
  cat /tmp/session_summary.txt          # live tail for decoded data
"""

from __future__ import annotations

import datetime
import math
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node

try:
    from sensors_interfaces.msg import UwbFrame as UWBFrameMsg
except ImportError:
    try:
        from sensors_interfaces.msg import UWBFrame as UWBFrameMsg  # type: ignore
    except ImportError:
        from sensors_interfaces.msg import UWB_Frame as UWBFrameMsg  # type: ignore

try:
    from sensors.sr250_protocol import parse_cir_udp_payload
except ImportError:
    from sr250_protocol import parse_cir_udp_payload


_RADAR_MODE_NAMES = {
    0x00: "CLOSE_DISTANCE",
    0x01: "MEDIUM_DISTANCE",
    0x02: "LONG_DISTANCE",
    0x03: "SHORT_RANGE",
}
_DATA_TYPE_NAMES = {
    0x00: "CIR_SAMPLES",
    0x01: "PRESENCE",
    0x20: "ANT_ISOLATION",
}


class UWBCIRInspector(Node):
    def __init__(self):
        super().__init__("uwb_cir_inspector")

        # Mirror the radar config params so the header accurately reports
        # what uwb_node sent to the STM32.
        self.declare_parameter("channel_number", 9)
        self.declare_parameter("preamble_code_index", 26)
        self.declare_parameter("radar_mode", 0x01)
        self.declare_parameter("rfri_ranging_interval_ms", 96)
        self.declare_parameter("rfri_slot_duration_rstu", 14400)
        self.declare_parameter("rfri_slots_per_rr", 4)
        self.declare_parameter("cir_num_samples", 128)
        self.declare_parameter("ant_rx_rxc_id", 0x01)
        self.declare_parameter("ant_rx_rxb_id", 0x02)
        self.declare_parameter("ant_rx_rxa_id", 0x00)
        self.declare_parameter("ant_tx_id", 0x01)
        self.declare_parameter("rx_gain_agc_mode", 0)
        self.declare_parameter("performance", 0x03)
        self.declare_parameter("drift_compensation", 0x0CCD)
        self.declare_parameter("presence_mode", 0x00)
        self.declare_parameter("stm32_ip", "192.168.1.10")
        self.declare_parameter("summary_path", "/tmp/session_summary.txt")

        self._summary_path = Path(self.get_parameter("summary_path").value)
        self._frame_count = 0

        self.create_subscription(UWBFrameMsg, "/uwb/frame_raw", self._on_frame, 10)

        self._write_header()
        self.get_logger().info(
            f"UWB CIR Inspector active. Writing summary to: {self._summary_path}"
        )

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _p(self, key: str):
        return self.get_parameter(key).value

    def _write_header(self):
        mode_id = int(self._p("radar_mode"))
        mode_name = _RADAR_MODE_NAMES.get(mode_id, f"0x{mode_id:02X}")
        rx_ids = (
            f"RXC={self._p('ant_rx_rxc_id')}"
            f"  RXB={self._p('ant_rx_rxb_id')}"
            f"  RXA={self._p('ant_rx_rxa_id')} (0 = disabled)"
        )

        lines = [
            "=" * 72,
            "  SR250 UWB  ─  Session Summary",
            f"  {datetime.datetime.now().strftime('%Y-%m-%d  %H:%M:%S')}",
            "=" * 72,
            "",
            "┌─── SENT FROM PC → STM32  (SET_CONFIG_FULL  →  START_RADAR) ─────────",
            f"│  Protocol envelope  : magic=0x55 0x57  version=0x01",
            f"│  STM32 IP           : {self._p('stm32_ip')}",
            f"│  STM32 command port : 37249  (UDP)",
            "│",
            f"│  channel_number      : {self._p('channel_number')}"
            f"   (UWB ch 9 ≈ 8.0 GHz centre)",
            f"│  preamble_code_index : {self._p('preamble_code_index')}",
            f"│  radar_mode          : {mode_id} ({mode_name})",
            f"│  rfri_interval_ms    : {self._p('rfri_ranging_interval_ms')} ms"
            f"   (frame repetition interval)",
            f"│  rfri_slot_duration  : {self._p('rfri_slot_duration_rstu')} RSTU",
            f"│  rfri_slots_per_rr   : {self._p('rfri_slots_per_rr')}",
            f"│  cir_num_samples     : {self._p('cir_num_samples')} taps"
            f"   (configured CIR tap count)",
            f"│  rx_antennas         : {rx_ids}",
            f"│  tx_antenna          : {self._p('ant_tx_id')}",
            f"│  rx_gain_agc_mode    : {self._p('rx_gain_agc_mode')}"
            f"   (0 = AGC, 1 = manual)",
            f"│  performance_flags   : 0x{int(self._p('performance')):02X}",
            f"│  drift_compensation  : 0x{int(self._p('drift_compensation')):04X}",
            f"│  presence_mode       : 0x{int(self._p('presence_mode')):02X}"
            f"   (0x00 = disabled, CIR-only mode)",
            "└──────────────────────────────────────────────────────────────────────",
            "",
            "┌─── RECEIVED FROM STM32 → PC  (RADAR_FRAME on UDP port 20000) ────────",
            "│  Full packet structure (little-endian):",
            "│",
            "│  [UDP envelope]",
            "│    magic[2]         : 0x55 0x57",
            "│    version[1]       : 0x01",
            "│    msg_type[1]      : 0x07  (RADAR_FRAME)",
            "│    seq[2]           : rolling counter",
            "│    payload_len[2]   : length of everything below",
            "│",
            "│  [RADAR_FRAME header  —  8 bytes]",
            "│    session_handle[4] : handle returned by SR250 for this session",
            "│    status[1]         : 0x00 = OK",
            "│    radar_data_type[1]: 0x00=CIR  0x01=PRESENCE  0x20=ANT_ISOLATION",
            "│    raw_len[2]        : length of UCI notification payload below",
            "│",
            "│  [UCI RADAR_RX_NTF payload  —  raw_len bytes]",
            "│    session_handle[4] : same as above",
            "│    status[1]",
            "│    data_type[1]",
            "│    num_samples[2]    : number of TX-RX antenna measurement pairs",
            "│    taps_per_sample[1]: total taps per pair (8 metadata + N CIR)",
            "│    rfu[1]",
            "│    --- per sample block (repeated num_samples times) ---",
            "│    metadata[32]      : cir_counter, timestamps, gain, antenna IDs,",
            "│                        rx_path, radar_mode, pulse info, CIR offset",
            "│    cir_data[N*bpt]   : complex taps (bpt = bytes per tap: 3 or 4)",
            "│      3-byte tap: real=int16_LE[0:2]  imag=sign_ext_int8[2]",
            "│      4-byte tap: real=int16_LE[0:2]  imag=int16_LE[2:4]",
            "└──────────────────────────────────────────────────────────────────────",
            "",
            "─── Decoded frames ───────────────────────────────────────────────────",
            "",
        ]
        self._summary_path.write_text("\n".join(lines))

    # ------------------------------------------------------------------
    # ROS2 subscriber callback
    # ------------------------------------------------------------------

    def _on_frame(self, msg: UWBFrameMsg):
        self._frame_count += 1
        raw = bytes(msg.raw_payload)
        data_type = msg.radar_data_type
        type_name = _DATA_TYPE_NAMES.get(data_type, f"0x{data_type:02X}")

        lines = [
            f"Frame #{self._frame_count:04d}  seq={msg.seq}"
            f"  type={type_name}"
            f"  session=0x{msg.session_handle:08X}"
            f"  status=0x{msg.status:02X}",
        ]

        if data_type == 0x00:  # CIR_SAMPLES
            self._decode_cir(raw, lines)
        else:
            lines.append(f"  raw_bytes={len(raw)}  (non-CIR notification)")
            self.get_logger().info(
                f"Frame #{self._frame_count}: type={type_name}  raw={len(raw)} B"
            )

        self._append(lines)

    def _decode_cir(self, raw: bytes, lines: list):
        try:
            parsed = parse_cir_udp_payload(raw)
        except Exception as exc:
            lines.append(f"  [CIR parse error: {exc}]")
            self.get_logger().warning(f"CIR parse error: {exc}")
            return

        ns = parsed["num_samples"]
        tps = parsed["taps_per_sample"]
        bpt = parsed["bytes_per_tap"]
        act = parsed.get("actual_cir_taps_per_block", "?")

        lines.append(
            f"  num_samples={ns}  taps_per_sample={tps}"
            f"  bytes_per_tap={bpt}  actual_cir_taps={act}"
        )

        for i, sample in enumerate(parsed["samples"]):
            m = sample["metadata"]
            taps = sample["taps"]
            peak = max((math.sqrt(r * r + im * im) for r, im in taps), default=0.0)
            first4 = "  ".join(f"({r:+d},{im:+d})" for r, im in taps[:4])
            lines += [
                f"  Sample[{i}]"
                f"  rx_ant={m['rx_antenna_id']}"
                f"  tx_ant={m['tx_antenna_id']}"
                f"  rx_path=0x{m['rx_path']:02X}"
                f"  radar_mode={m['radar_mode']}",
                f"    cir_counter={m['cir_counter']}"
                f"  rx_ts={m['rx_timestamp']}"
                f"  tx_ts={m['tx_timestamp']}",
                f"    tx_gain={m['tx_gain_index']}"
                f"  rx_gain={m['rx_gain_index']}"
                f"  cir_offset={m['cir_start_offset']}"
                f"  num_pulses={m['num_pulses_used']}",
                f"    num_cir_taps={sample['num_cir_taps']}"
                f"  peak_magnitude={peak:.1f}",
                f"    first 4 taps (real, imag): {first4}",
            ]

        self.get_logger().info(
            f"CIR #{self._frame_count}:"
            f"  session=0x{parsed['session_handle']:08X}"
            f"  samples={ns}"
            f"  cir_taps={act}"
            f"  peak[0]={max((math.sqrt(r*r+im*im) for r,im in parsed['samples'][0]['taps']), default=0.0):.1f}"
            if parsed["samples"] else
            f"CIR #{self._frame_count}: no samples"
        )

    def _append(self, lines: list):
        with self._summary_path.open("a") as fh:
            fh.write("\n".join(lines) + "\n\n")


def main():
    rclpy.init()
    node: Optional[UWBCIRInspector] = None
    try:
        node = UWBCIRInspector()
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

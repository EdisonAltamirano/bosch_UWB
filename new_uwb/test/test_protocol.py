"""Offline tests for new_uwb.protocol — no ROS2, no hardware required.

Run with: python -m unittest new_uwb.test.test_protocol -v
(from the new_uwb/ package directory, or anywhere new_uwb/ is on sys.path)
"""
from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from new_uwb import protocol as p  # noqa: E402


class TestBuilders(unittest.TestCase):
    def test_build_start_session_packet_layout(self):
        packet = p.build_start_session_packet(setting_idx=0, duration_ms=10000)
        self.assertEqual(packet, bytes.fromhex("01" "00" "10270000"))
        self.assertEqual(len(packet), 6)

    def test_build_start_session_packet_nonzero_preset(self):
        packet = p.build_start_session_packet(setting_idx=1, duration_ms=1000)
        cmd_id, setting_idx, duration_ms = struct.unpack("<BBI", packet)
        self.assertEqual(cmd_id, p.CMD_START_SESSION)
        self.assertEqual(setting_idx, 1)
        self.assertEqual(duration_ms, 1000)

    def test_build_system_reset_packet(self):
        packet = p.build_system_reset_packet()
        self.assertEqual(packet, bytes([p.CMD_SYSTEM_COMMAND, p.CMD_SYSTEM_RESET]))


class TestAckParser(unittest.TestCase):
    def test_ack_for_start_session(self):
        ack = p.parse_ack_packet(bytes([0x03, 0x01]))
        self.assertEqual(ack["recv_cmd_id"], p.CMD_START_SESSION)

    def test_ack_wrong_cmd_id_rejected(self):
        with self.assertRaises(ValueError):
            p.parse_ack_packet(bytes([0x04, 0x01]))

    def test_ack_bad_length_rejected(self):
        with self.assertRaises(ValueError):
            p.parse_ack_packet(bytes([0x03]))
        with self.assertRaises(ValueError):
            p.parse_ack_packet(bytes([0x03, 0x01, 0x00]))


class TestErrorParser(unittest.TestCase):
    def test_known_error_id(self):
        err = p.parse_error_packet(bytes([0x04, 0x0F]))
        self.assertEqual(err["err_name"], "ERR_UNKNOWN_USER_CMD_RECEIVED")

    def test_unknown_error_id_falls_back_to_hex_name(self):
        err = p.parse_error_packet(bytes([0x04, 0xFE]))
        self.assertEqual(err["err_name"], "UNKNOWN_0xFE")

    def test_error_bad_length_rejected(self):
        with self.assertRaises(ValueError):
            p.parse_error_packet(bytes([0x04]))


class TestSystemStateParser(unittest.TestCase):
    def test_system_ok(self):
        state = p.parse_system_state_packet(bytes([0x06, 0x00]))
        self.assertEqual(state["state_name"], "SYSTEM_STATE_OK")

    def test_radar_session_done(self):
        state = p.parse_system_state_packet(bytes([0x06, 0x02]))
        self.assertEqual(state["state_name"], "SYSTEM_STATE_RADAR_SESSION_DONE")

    def test_wrong_cmd_id_rejected(self):
        with self.assertRaises(ValueError):
            p.parse_system_state_packet(bytes([0x03, 0x00]))


class TestCIRFragmentParser(unittest.TestCase):
    def test_single_fragment_valid_data_len(self):
        data = bytes(range(20))
        packet = bytes([p.CMD_CIR_REPORT, 0x01]) + struct.pack("<H", len(data)) + data
        frag = p.parse_cir_fragment_packet(packet)
        self.assertTrue(frag["last_fragment"])
        self.assertEqual(frag["data"], data)
        self.assertEqual(frag["data_len"], len(data))

    def test_non_last_fragment_flag(self):
        data = bytes(range(10))
        packet = bytes([p.CMD_CIR_REPORT, 0x00]) + struct.pack("<H", len(data)) + data
        frag = p.parse_cir_fragment_packet(packet)
        self.assertFalse(frag["last_fragment"])

    def test_too_short_rejected(self):
        with self.assertRaises(ValueError):
            p.parse_cir_fragment_packet(bytes([p.CMD_CIR_REPORT, 0x01, 0x00]))

    def test_data_len_mismatch_rejected(self):
        # declares 50 bytes of data but only supplies 5
        packet = bytes([p.CMD_CIR_REPORT, 0x01]) + struct.pack("<H", 50) + bytes(5)
        with self.assertRaises(ValueError):
            p.parse_cir_fragment_packet(packet)

    def test_data_len_over_max_rejected(self):
        packet = bytes([p.CMD_CIR_REPORT, 0x01]) + struct.pack("<H", p.MAX_DATA_PER_ETH_PACKET + 1)
        with self.assertRaises(ValueError):
            p.parse_cir_fragment_packet(packet)

    def test_wrong_cmd_id_rejected(self):
        packet = bytes([p.CMD_UDP_ETH_ACK, 0x01]) + struct.pack("<H", 0)
        with self.assertRaises(ValueError):
            p.parse_cir_fragment_packet(packet)


class TestCIRReassembler(unittest.TestCase):
    def _fragment(self, data: bytes, last: bool) -> dict:
        packet = bytes([p.CMD_CIR_REPORT, 0x01 if last else 0x00]) + struct.pack("<H", len(data)) + data
        return p.parse_cir_fragment_packet(packet)

    def test_single_fragment_frame(self):
        r = p.CIRReassembler()
        payload = bytes(range(30))
        result = r.feed(self._fragment(payload, last=True))
        self.assertEqual(result, payload)
        self.assertEqual(len(r), 0)

    def test_multi_fragment_frame_reassembles_in_order(self):
        r = p.CIRReassembler()
        part1 = bytes(range(0, 50))
        part2 = bytes(range(50, 100))
        part3 = bytes(range(100, 130))

        self.assertIsNone(r.feed(self._fragment(part1, last=False)))
        self.assertGreater(len(r), 0)
        self.assertIsNone(r.feed(self._fragment(part2, last=False)))
        result = r.feed(self._fragment(part3, last=True))

        self.assertEqual(result, part1 + part2 + part3)
        self.assertEqual(len(r), 0)  # buffer cleared after completion

    def test_reassembler_reusable_after_reset(self):
        r = p.CIRReassembler()
        r.feed(self._fragment(bytes(10), last=False))
        r.reset()
        self.assertEqual(len(r), 0)
        result = r.feed(self._fragment(bytes(5), last=True))
        self.assertEqual(result, bytes(5))

    def test_max_size_fragment_matches_firmware_chunking(self):
        # Mirrors nucleo_udp_send()'s CMD_CIR_REPORT loop in udp_server.c: a
        # payload longer than MAX_DATA_PER_ETH_PACKET splits into a
        # non-last MAX_DATA_PER_ETH_PACKET-sized fragment plus a last
        # remainder fragment.
        full_payload = bytes((i % 256) for i in range(p.MAX_DATA_PER_ETH_PACKET + 37))
        first = full_payload[: p.MAX_DATA_PER_ETH_PACKET]
        rest = full_payload[p.MAX_DATA_PER_ETH_PACKET:]

        r = p.CIRReassembler()
        self.assertIsNone(r.feed(self._fragment(first, last=False)))
        result = r.feed(self._fragment(rest, last=True))
        self.assertEqual(result, full_payload)


class TestIdentifyPacket(unittest.TestCase):
    def test_known_commands(self):
        self.assertEqual(p.identify_packet(bytes([0x01])), "START_SESSION")
        self.assertEqual(p.identify_packet(bytes([0x02])), "CIR_REPORT")
        self.assertEqual(p.identify_packet(bytes([0x03])), "ACK")
        self.assertEqual(p.identify_packet(bytes([0x04])), "ERROR_REPORT")
        self.assertEqual(p.identify_packet(bytes([0x05])), "SYSTEM_COMMAND")
        self.assertEqual(p.identify_packet(bytes([0x06])), "SYSTEM_STATE")

    def test_unknown_command(self):
        self.assertEqual(p.identify_packet(bytes([0xAB])), "UNKNOWN_0xAB")

    def test_empty_packet(self):
        self.assertEqual(p.identify_packet(b""), "EMPTY")


if __name__ == "__main__":
    unittest.main()

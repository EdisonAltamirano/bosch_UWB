"""Wire protocol for the `uwb_sw` (NXP board) firmware — the live `CMD_*` scheme.

Verified against `uwb_sw/Core/Src/udp_server.c` and `uwb_sw/Core/Inc/udp_server.h`
(see debug_logs/D0_firmware_inventory.md and NEW_UWB_PLAN_ADDENDUM.md). This is
NOT the TLV envelope used by legacy_tlv/boshUWBSTM32 — do not mix them up.

Once a CMD_CIR_REPORT fragment train is reassembled with CIRReassembler, the
resulting raw payload is the same raw SR250 UCI RADAR_RX_NTF payload that
legacy_tlv sends (confirmed: both firmwares pass the notification handler's
`payload`/`len` straight into their respective transport-send calls without
reformatting). That means the reassembled payload can be parsed with
`sensors.sr250_protocol.parse_cir_udp_payload` unmodified — see
new_uwb_udp_frame_publisher.py.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
import struct

# ---------------------------------------------------------------------------
# Constants — copied verbatim from uwb_sw/Core/Inc/udp_server.h
# ---------------------------------------------------------------------------

NUCLEO_ETH_PORT = 37249       # PC -> board command port
NUCLEO_REMOTE_PORT = 20000    # board -> PC data port (ACK/ERROR/SYSTEM_STATE/CIR, all of it)
MAX_DATA_PER_ETH_PACKET = 1184

CMD_START_SESSION = 0x01
CMD_CIR_REPORT = 0x02
CMD_UDP_ETH_ACK = 0x03
CMD_ERROR_REPORT = 0x04
CMD_SYSTEM_COMMAND = 0x05
CMD_SYSTEM_STATE = 0x06

CMD_SYSTEM_RESET = 0x01

CMD_SYSTEM_STATE_OK = 0x00
CMD_SYSTEM_STATE_NOK = 0x01
CMD_SYSTEM_STATE_RADAR_SESSION_DONE = 0x02

ERROR_NAMES = {
    0x03: "ERR_INIT_UWB_SUBSYSTEM_FAILED",
    0x05: "ERR_INIT_UWB_RADAR_FAILED",
    0x07: "ERR_CONFIG_UWB_RADAR_FAILED",
    0x09: "ERR_START_UWB_RADAR_FAILED",
    0x0D: "ERR_UWB_RADAR_SESSION_STOP_FAILED",
    0x0E: "ERR_UWB_RADAR_SESSION_DEINIT_FAILED",
    0x0F: "ERR_UNKNOWN_USER_CMD_RECEIVED",
    0x10: "ERR_UNKNOWN_SYS_CMD_RECEIVED",
}

SYSTEM_STATE_NAMES = {
    CMD_SYSTEM_STATE_OK: "SYSTEM_STATE_OK",
    CMD_SYSTEM_STATE_NOK: "SYSTEM_STATE_NOK",
    CMD_SYSTEM_STATE_RADAR_SESSION_DONE: "SYSTEM_STATE_RADAR_SESSION_DONE",
}

_CMD_NAMES = {
    CMD_START_SESSION: "START_SESSION",
    CMD_CIR_REPORT: "CIR_REPORT",
    CMD_UDP_ETH_ACK: "ACK",
    CMD_ERROR_REPORT: "ERROR_REPORT",
    CMD_SYSTEM_COMMAND: "SYSTEM_COMMAND",
    CMD_SYSTEM_STATE: "SYSTEM_STATE",
}


def identify_packet(packet: bytes) -> str:
    """Best-effort command name for logging; does not validate the payload."""
    if not packet:
        return "EMPTY"
    return _CMD_NAMES.get(packet[0], f"UNKNOWN_0x{packet[0]:02X}")


# ---------------------------------------------------------------------------
# PC -> board builders
# ---------------------------------------------------------------------------

def build_start_session_packet(setting_idx: int, duration_ms: int) -> bytes:
    """udp_input_cmd_t: <cmd_id u8><setting_idx u8><duration_in_ms u32 LE>."""
    return struct.pack("<BBI", CMD_START_SESSION, setting_idx & 0xFF, duration_ms & 0xFFFFFFFF)


def build_system_reset_packet() -> bytes:
    """CMD_SYSTEM_COMMAND with sub-command CMD_SYSTEM_RESET: <cmd_id u8><sub_cmd u8>."""
    return struct.pack("<BB", CMD_SYSTEM_COMMAND, CMD_SYSTEM_RESET)


# ---------------------------------------------------------------------------
# Board -> PC parsers
# ---------------------------------------------------------------------------

def parse_ack_packet(packet: bytes) -> dict:
    """udp_input_ack_cmd_t: <cmd_id u8=0x03><recv_cmd_id u8>."""
    if len(packet) != 2:
        raise ValueError(f"ACK packet must be 2 bytes, got {len(packet)}")
    cmd_id, recv_cmd_id = struct.unpack("<BB", packet)
    if cmd_id != CMD_UDP_ETH_ACK:
        raise ValueError(f"Not an ACK packet: cmd_id=0x{cmd_id:02X}")
    return {"cmd_id": cmd_id, "recv_cmd_id": recv_cmd_id}


def parse_error_packet(packet: bytes) -> dict:
    """udp_err_report_cmd_t: <cmd_id u8=0x04><err_id u8>."""
    if len(packet) != 2:
        raise ValueError(f"ERROR packet must be 2 bytes, got {len(packet)}")
    cmd_id, err_id = struct.unpack("<BB", packet)
    if cmd_id != CMD_ERROR_REPORT:
        raise ValueError(f"Not an ERROR packet: cmd_id=0x{cmd_id:02X}")
    return {
        "cmd_id": cmd_id,
        "err_id": err_id,
        "err_name": ERROR_NAMES.get(err_id, f"UNKNOWN_0x{err_id:02X}"),
    }


def parse_system_state_packet(packet: bytes) -> dict:
    """Same 2-byte shape as ERROR: <cmd_id u8=0x06><state_id u8>.

    Only reaches the wire once the D1 nucleo_udp_send() switch fix is flashed —
    see debug_logs/D1_cubeide_build.md. Absence of this packet on an unpatched
    board is expected, not a parser bug.
    """
    if len(packet) != 2:
        raise ValueError(f"SYSTEM_STATE packet must be 2 bytes, got {len(packet)}")
    cmd_id, state_id = struct.unpack("<BB", packet)
    if cmd_id != CMD_SYSTEM_STATE:
        raise ValueError(f"Not a SYSTEM_STATE packet: cmd_id=0x{cmd_id:02X}")
    return {
        "cmd_id": cmd_id,
        "state_id": state_id,
        "state_name": SYSTEM_STATE_NAMES.get(state_id, f"UNKNOWN_0x{state_id:02X}"),
    }


def parse_cir_fragment_packet(packet: bytes) -> dict:
    """udp_cir_fragment_t: <cmd_id u8=0x02><last_fragment u8><data_len u16 LE><data>."""
    if len(packet) < 4:
        raise ValueError(f"CIR fragment too short: {len(packet)} bytes")
    cmd_id, last_fragment, data_len = struct.unpack_from("<BBH", packet, 0)
    if cmd_id != CMD_CIR_REPORT:
        raise ValueError(f"Not a CIR fragment: cmd_id=0x{cmd_id:02X}")
    if data_len > MAX_DATA_PER_ETH_PACKET:
        raise ValueError(
            f"CIR fragment data_len {data_len} exceeds MAX_DATA_PER_ETH_PACKET "
            f"{MAX_DATA_PER_ETH_PACKET}"
        )
    data = packet[4:4 + data_len]
    if len(data) != data_len:
        raise ValueError(
            f"CIR fragment length mismatch: declared {data_len}, got {len(data)}"
        )
    return {
        "cmd_id": cmd_id,
        "last_fragment": bool(last_fragment),
        "data_len": data_len,
        "data": data,
    }


class CIRReassembler:
    """Buffers CMD_CIR_REPORT fragments until last_fragment=1, then yields the
    full raw NTF payload. One instance per in-flight frame; a malformed
    fragment (raises in parse_cir_fragment_packet) should be handled by the
    caller resetting the buffer, since a dropped/corrupt fragment makes the
    rest of the train unusable.
    """

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, fragment: dict) -> Optional[bytes]:
        self._buffer.extend(fragment["data"])
        if fragment["last_fragment"]:
            payload = bytes(self._buffer)
            self._buffer = bytearray()
            return payload
        return None

    def reset(self) -> None:
        self._buffer = bytearray()

    def __len__(self) -> int:
        return len(self._buffer)

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Any, Dict, Iterable, List, Optional, Tuple
import struct
import re
from pathlib import Path


MAGIC = b"\x55\x57"
PROTOCOL_VERSION = 0x01

MSG_TYPE_SET_CONFIG_FULL = 0x01
MSG_TYPE_SET_PARAMS_PARTIAL = 0x02
MSG_TYPE_START_RADAR = 0x03
MSG_TYPE_STOP_RADAR = 0x04
MSG_TYPE_ACK = 0x05
MSG_TYPE_ERROR = 0x06
MSG_TYPE_RADAR_FRAME = 0x07
MSG_TYPE_RADAR_FRAME_CHUNK = 0x08  # chunked large frame — reassemble before processing

CMD_START_SESSION = 0x01
CMD_CIR_REPORT = 0x02
CMD_UDP_ETH_ACK = 0x03
CMD_ERROR_REPORT = 0x04

TAG_CHANNEL_NUMBER = 0x0001
TAG_PREAMBLE_CODE_INDEX = 0x0002
TAG_ANTENNAS_CONFIG_RX = 0x0003
TAG_ANTENNAS_CONFIG_TX = 0x0004
TAG_RADAR_MODE = 0x0005
TAG_RADAR_SINGLE_FRAME_NTF = 0x0006
TAG_RADAR_RFRI = 0x0007
TAG_RADAR_CIR_NUM_SAMPLES = 0x0008
TAG_RADAR_RX_GAIN = 0x0009
TAG_RADAR_CIR_START_OFFSET = 0x000A
TAG_RADAR_PERFORMANCE = 0x000B
TAG_RADAR_DRIFT_COMPENSATION = 0x000C
TAG_RADAR_PRESENCE_DET_CFG = 0x000D

ERROR_INVALID_MAGIC = 0x0001
ERROR_INVALID_VERSION = 0x0002
ERROR_INVALID_LENGTH = 0x0003
ERROR_INVALID_TLV = 0x0004
ERROR_MISSING_REQUIRED_FIELD = 0x0005
ERROR_INVALID_STATE = 0x0006
ERROR_UCI_COMMAND_FAILED = 0x0007
ERROR_UNSUPPORTED_TAG = 0x0008

UWB_SW_ERROR_NAMES = {
    0x03: "ERR_INIT_UWB_SUBSYSTEM_FAILED",
    0x05: "ERR_INIT_UWB_RADAR_FAILED",
    0x07: "ERR_CONFIG_UWB_RADAR_FAILED",
    0x09: "ERR_START_UWB_RADAR_FAILED",
    0x0D: "ERR_UWB_RADAR_SESSION_STOP_FAILED",
    0x0E: "ERR_UWB_RADAR_SESSION_DEINIT_FAILED",
    0x0F: "ERR_UNKNOWN_USER_CMD_RECEIVED",
}

METADATA_BYTES = 32
METADATA_TAPS = 8


@dataclass
class CIRMetadata:
    session_handle: int
    status: int
    radar_data_type: int
    num_samples: int
    taps_per_sample: int
    rfu: int
    sample_index: int
    cir_counter: int
    rx_timestamp: int
    tx_timestamp: int
    tap_resolution_bits: int
    tap_resolution_cm: int
    tx_gain_index: int
    rx_gain_index: int
    tx_antenna_id: int
    rx_antenna_id: int
    rx_path: int
    radar_mode: int
    inter_pulse_distance_in_ns: int
    correlation_bit_shift: int
    log_2_delta_L: int
    tx_pulse_shape_id: int
    dc_freeze_enabled: bool
    single_pin: bool
    nsync_by_8: int
    peak_real_cir: int
    num_pulses_used: int
    cir_start_offset_fractional: int
    cir_start_offset: int


def le16(b: bytes) -> int:
    return struct.unpack_from("<H", b)[0]


def le32(b: bytes) -> int:
    return struct.unpack_from("<I", b)[0]


def sign_extend_int8(x: int) -> int:
    return x - 256 if x > 127 else x


def pack_envelope(msg_type: int, seq: int, payload: bytes) -> bytes:
    return MAGIC + struct.pack("<BBHH", PROTOCOL_VERSION, msg_type, seq, len(payload)) + payload


def unpack_envelope(packet: bytes) -> Dict[str, Any]:
    if len(packet) < 8:
        raise ValueError(f"Packet too short for envelope: {len(packet)} bytes")
    if packet[0:2] != MAGIC:
        raise ValueError(f"Invalid magic: {packet[0:2].hex(' ')}")

    version, msg_type, seq, payload_len = struct.unpack_from("<BBHH", packet, 2)
    if version != PROTOCOL_VERSION:
        raise ValueError(f"Unsupported version: {version}")

    if len(packet) != 8 + payload_len:
        raise ValueError(
            f"Envelope payload_len mismatch: expected {payload_len}, got {len(packet) - 8}"
        )

    return {
        "version": version,
        "msg_type": msg_type,
        "seq": seq,
        "payload_len": payload_len,
        "payload": packet[8:],
    }


def pack_tlv(tag: int, value: bytes) -> bytes:
    return struct.pack("<HH", tag, len(value)) + value


def unpack_tlvs(payload: bytes) -> List[Tuple[int, bytes]]:
    tlvs: List[Tuple[int, bytes]] = []
    offset = 0
    while offset < len(payload):
        if offset + 4 > len(payload):
            raise ValueError("Truncated TLV header")
        tag, length = struct.unpack_from("<HH", payload, offset)
        offset += 4
        if offset + length > len(payload):
            raise ValueError(f"Truncated TLV value for tag 0x{tag:04X}")
        tlvs.append((tag, payload[offset:offset + length]))
        offset += length
    return tlvs


def encode_u8(tag: int, value: int) -> bytes:
    return pack_tlv(tag, struct.pack("<B", value))


def encode_u16(tag: int, value: int) -> bytes:
    return pack_tlv(tag, struct.pack("<H", value))


def encode_u32(tag: int, value: int) -> bytes:
    return pack_tlv(tag, struct.pack("<I", value))


def encode_bytes(tag: int, value: Iterable[int] | bytes | bytearray) -> bytes:
    if isinstance(value, (bytes, bytearray)):
        data = bytes(value)
    else:
        data = bytes(value)
    return pack_tlv(tag, data)


def build_radar_config_tlvs(config: Dict[str, Any]) -> bytes:
    tlvs: List[bytes] = [
        encode_u8(TAG_CHANNEL_NUMBER, config["channel_number"]),
        encode_u8(TAG_PREAMBLE_CODE_INDEX, config["preamble_code_index"]),
        encode_bytes(
            TAG_ANTENNAS_CONFIG_RX,
            [
                config.get("antennas_config_rx_mode", 0x02),
                0x03,
                config["ant_rx_rxc_id"],
                config["ant_rx_rxb_id"],
                config["ant_rx_rxa_id"],
            ],
        ),
        encode_bytes(
            TAG_ANTENNAS_CONFIG_TX,
            [
                0x01,
                config["ant_tx_id"],
            ],
        ),
        encode_u8(TAG_RADAR_MODE, config["radar_mode"]),
        encode_u8(TAG_RADAR_SINGLE_FRAME_NTF, config["single_frame_ntf"]),
        encode_bytes(
            TAG_RADAR_RFRI,
            struct.pack(
                "<IHB",
                config["rfri_ranging_interval_ms"],
                config["rfri_slot_duration_rstu"],
                config["rfri_slots_per_rr"],
            ),
        ),
        encode_u8(TAG_RADAR_CIR_NUM_SAMPLES, config["cir_num_samples"]),
        encode_bytes(
            TAG_RADAR_RX_GAIN,
            [
                config["rx_gain_agc_mode"],
                config["rx_gain_rxa"],
                config["rx_gain_rxb"],
                config["rx_gain_rxc"],
            ],
        ),
        encode_bytes(
            TAG_RADAR_CIR_START_OFFSET,
            struct.pack(
                "<HHH",
                config["cir_start_offset_rxc"],
                config["cir_start_offset_rxb"],
                config["cir_start_offset_rxa"],
            ),
        ),
        encode_u8(TAG_RADAR_PERFORMANCE, config["performance"]),
        encode_u16(TAG_RADAR_DRIFT_COMPENSATION, config["drift_compensation"]),
        encode_bytes(
            TAG_RADAR_PRESENCE_DET_CFG,
            struct.pack(
                "<BBBBHHHbb",
                config["presence_mode"],
                config["presence_periodic_report"],
                config["presence_sensitivity_q4_4"],
                config["presence_gpio_notify"],
                config["presence_distance_min_cm"],
                config["presence_distance_max_cm"],
                config["presence_hold_delay_ms"],
                config["presence_angle_min_deg"],
                config["presence_angle_max_deg"],
            ),
        ),
    ]
    return b"".join(tlvs)


def build_partial_radar_config_tlvs(config: Dict[str, Any]) -> bytes:
    tlvs: List[bytes] = []

    def require_keys(*keys: str) -> None:
        missing = [key for key in keys if key not in config]
        if missing:
            raise KeyError(f"Missing keys for partial TLV group: {missing}")

    if "channel_number" in config:
        tlvs.append(encode_u8(TAG_CHANNEL_NUMBER, config["channel_number"]))

    if "preamble_code_index" in config:
        tlvs.append(encode_u8(TAG_PREAMBLE_CODE_INDEX, config["preamble_code_index"]))

    if any(
        key in config
        for key in (
            "antennas_config_rx_mode",
            "ant_rx_rxc_id",
            "ant_rx_rxb_id",
            "ant_rx_rxa_id",
        )
    ):
        require_keys(
            "antennas_config_rx_mode",
            "ant_rx_rxc_id",
            "ant_rx_rxb_id",
            "ant_rx_rxa_id",
        )
        tlvs.append(
            encode_bytes(
                TAG_ANTENNAS_CONFIG_RX,
                [
                    config["antennas_config_rx_mode"],
                    0x03,
                    config["ant_rx_rxc_id"],
                    config["ant_rx_rxb_id"],
                    config["ant_rx_rxa_id"],
                ],
            )
        )

    if "ant_tx_id" in config:
        tlvs.append(encode_bytes(TAG_ANTENNAS_CONFIG_TX, [0x01, config["ant_tx_id"]]))

    if "radar_mode" in config:
        tlvs.append(encode_u8(TAG_RADAR_MODE, config["radar_mode"]))

    if "single_frame_ntf" in config:
        tlvs.append(encode_u8(TAG_RADAR_SINGLE_FRAME_NTF, config["single_frame_ntf"]))

    if any(
        key in config
        for key in (
            "rfri_ranging_interval_ms",
            "rfri_slot_duration_rstu",
            "rfri_slots_per_rr",
        )
    ):
        require_keys(
            "rfri_ranging_interval_ms",
            "rfri_slot_duration_rstu",
            "rfri_slots_per_rr",
        )
        tlvs.append(
            encode_bytes(
                TAG_RADAR_RFRI,
                struct.pack(
                    "<IHB",
                    config["rfri_ranging_interval_ms"],
                    config["rfri_slot_duration_rstu"],
                    config["rfri_slots_per_rr"],
                ),
            )
        )

    if "cir_num_samples" in config:
        tlvs.append(encode_u8(TAG_RADAR_CIR_NUM_SAMPLES, config["cir_num_samples"]))

    if any(
        key in config
        for key in (
            "rx_gain_agc_mode",
            "rx_gain_rxa",
            "rx_gain_rxb",
            "rx_gain_rxc",
        )
    ):
        require_keys(
            "rx_gain_agc_mode",
            "rx_gain_rxa",
            "rx_gain_rxb",
            "rx_gain_rxc",
        )
        tlvs.append(
            encode_bytes(
                TAG_RADAR_RX_GAIN,
                [
                    config["rx_gain_agc_mode"],
                    config["rx_gain_rxa"],
                    config["rx_gain_rxb"],
                    config["rx_gain_rxc"],
                ],
            )
        )

    if any(
        key in config
        for key in (
            "cir_start_offset_rxc",
            "cir_start_offset_rxb",
            "cir_start_offset_rxa",
        )
    ):
        require_keys(
            "cir_start_offset_rxc",
            "cir_start_offset_rxb",
            "cir_start_offset_rxa",
        )
        tlvs.append(
            encode_bytes(
                TAG_RADAR_CIR_START_OFFSET,
                struct.pack(
                    "<HHH",
                    config["cir_start_offset_rxc"],
                    config["cir_start_offset_rxb"],
                    config["cir_start_offset_rxa"],
                ),
            )
        )

    if "performance" in config:
        tlvs.append(encode_u8(TAG_RADAR_PERFORMANCE, config["performance"]))

    if "drift_compensation" in config:
        tlvs.append(encode_u16(TAG_RADAR_DRIFT_COMPENSATION, config["drift_compensation"]))

    if any(
        key in config
        for key in (
            "presence_mode",
            "presence_periodic_report",
            "presence_sensitivity_q4_4",
            "presence_gpio_notify",
            "presence_distance_min_cm",
            "presence_distance_max_cm",
            "presence_hold_delay_ms",
            "presence_angle_min_deg",
            "presence_angle_max_deg",
        )
    ):
        require_keys(
            "presence_mode",
            "presence_periodic_report",
            "presence_sensitivity_q4_4",
            "presence_gpio_notify",
            "presence_distance_min_cm",
            "presence_distance_max_cm",
            "presence_hold_delay_ms",
            "presence_angle_min_deg",
            "presence_angle_max_deg",
        )
        tlvs.append(
            encode_bytes(
                TAG_RADAR_PRESENCE_DET_CFG,
                struct.pack(
                    "<BBBBHHHbb",
                    config["presence_mode"],
                    config["presence_periodic_report"],
                    config["presence_sensitivity_q4_4"],
                    config["presence_gpio_notify"],
                    config["presence_distance_min_cm"],
                    config["presence_distance_max_cm"],
                    config["presence_hold_delay_ms"],
                    config["presence_angle_min_deg"],
                    config["presence_angle_max_deg"],
                ),
            )
        )

    return b"".join(tlvs)


def build_set_config_full_packet(seq: int, config: Dict[str, Any]) -> bytes:
    return pack_envelope(MSG_TYPE_SET_CONFIG_FULL, seq, build_radar_config_tlvs(config))


def build_set_params_partial_packet(seq: int, tlv_payload: bytes) -> bytes:
    return pack_envelope(MSG_TYPE_SET_PARAMS_PARTIAL, seq, tlv_payload)


def build_start_radar_packet(seq: int, duration_ms: int = 0) -> bytes:
    payload = struct.pack("<I", duration_ms) if duration_ms > 0 else b""
    return pack_envelope(MSG_TYPE_START_RADAR, seq, payload)


def build_stop_radar_packet(seq: int) -> bytes:
    return pack_envelope(MSG_TYPE_STOP_RADAR, seq, b"")


def build_uwb_sw_start_session_packet(setting_idx: int, duration_in_ms: int) -> bytes:
    return struct.pack("<BBI", CMD_START_SESSION, setting_idx, duration_in_ms)


def parse_ack_payload(payload: bytes) -> Dict[str, Any]:
    if len(payload) < 3:
        raise ValueError("ACK payload too short")
    acked_msg_type, seq = struct.unpack_from("<BH", payload, 0)
    flags = payload[3:] if len(payload) > 3 else b""
    return {
        "acked_msg_type": acked_msg_type,
        "seq": seq,
        "flags": flags,
    }


def parse_error_payload(payload: bytes) -> Dict[str, Any]:
    if len(payload) < 5:
        raise ValueError("ERROR payload too short")
    failed_msg_type, seq, error_code = struct.unpack_from("<BHH", payload, 0)
    detail = payload[5:] if len(payload) > 5 else b""
    return {
        "failed_msg_type": failed_msg_type,
        "seq": seq,
        "error_code": error_code,
        "detail": detail,
    }


def parse_uwb_sw_ack_packet(packet: bytes) -> Dict[str, Any]:
    if len(packet) != 2:
        raise ValueError(f"uwb_sw ACK packet must be 2 bytes, got {len(packet)}")
    cmd_id, recv_cmd_id = struct.unpack("<BB", packet)
    if cmd_id != CMD_UDP_ETH_ACK:
        raise ValueError(f"Not a uwb_sw ACK packet: cmd_id=0x{cmd_id:02X}")
    return {
        "cmd_id": cmd_id,
        "recv_cmd_id": recv_cmd_id,
    }


def parse_uwb_sw_error_packet(packet: bytes) -> Dict[str, Any]:
    if len(packet) != 2:
        raise ValueError(f"uwb_sw ERROR packet must be 2 bytes, got {len(packet)}")
    cmd_id, err_id = struct.unpack("<BB", packet)
    if cmd_id != CMD_ERROR_REPORT:
        raise ValueError(f"Not a uwb_sw ERROR packet: cmd_id=0x{cmd_id:02X}")
    return {
        "cmd_id": cmd_id,
        "err_id": err_id,
        "err_name": UWB_SW_ERROR_NAMES.get(err_id, "UNKNOWN_ERROR"),
    }


def parse_uwb_sw_cir_fragment_packet(packet: bytes) -> Dict[str, Any]:
    if len(packet) < 4:
        raise ValueError(f"uwb_sw CIR fragment too short: {len(packet)} bytes")

    cmd_id = packet[0]
    if cmd_id != CMD_CIR_REPORT:
        raise ValueError(f"Not a uwb_sw CIR fragment: cmd_id=0x{cmd_id:02X}")

    last_fragment = packet[1]
    data_len = le16(packet[2:4])
    if len(packet) != 4 + data_len:
        raise ValueError(
            f"uwb_sw CIR fragment length mismatch: expected {data_len}, got {len(packet) - 4}"
        )

    return {
        "cmd_id": cmd_id,
        "last_fragment": bool(last_fragment),
        "data_len": data_len,
        "data": packet[4:],
    }


def parse_radar_frame_chunk_payload(payload: bytes) -> Dict[str, Any]:
    if len(payload) < 4:
        raise ValueError(f"RADAR_FRAME_CHUNK payload too short: {len(payload)} bytes")
    frame_id = struct.unpack_from("<H", payload, 0)[0]
    chunk_idx = payload[2]
    total_chunks = payload[3]
    return {
        "frame_id": frame_id,
        "chunk_idx": chunk_idx,
        "total_chunks": total_chunks,
        "chunk_data": payload[4:],
    }


def parse_radar_frame_payload(payload: bytes) -> Dict[str, Any]:
    if len(payload) < 8:
        raise ValueError("RADAR_FRAME payload too short")
    session_handle = le32(payload[0:4])
    status = payload[4]
    radar_data_type = payload[5]
    raw_len = le16(payload[6:8])
    if len(payload) != 8 + raw_len:
        raise ValueError(
            f"RADAR_FRAME raw_len mismatch: expected {raw_len}, got {len(payload) - 8}"
        )
    raw_payload = payload[8:]
    return {
        "session_handle": session_handle,
        "status": status,
        "radar_data_type": radar_data_type,
        "raw_payload_len": raw_len,
        "raw_payload": raw_payload,
    }


def parse_metadata_block(
    metadata_block: bytes,
    session_handle: int,
    status: int,
    radar_data_type: int,
    num_samples: int,
    taps_per_sample: int,
    rfu: int,
    sample_index: int,
) -> CIRMetadata:
    return CIRMetadata(
        session_handle=session_handle,
        status=status,
        radar_data_type=radar_data_type,
        num_samples=num_samples,
        taps_per_sample=taps_per_sample,
        rfu=rfu,
        sample_index=sample_index,
        cir_counter=le32(metadata_block[0:4]),
        rx_timestamp=le32(metadata_block[4:8]),
        tx_timestamp=le32(metadata_block[12:16]),
        tap_resolution_bits=metadata_block[11] & 0x0C,
        tap_resolution_cm=metadata_block[11] & 0x03,
        tx_gain_index=metadata_block[9],
        rx_gain_index=metadata_block[8],
        tx_antenna_id=metadata_block[19],
        rx_antenna_id=metadata_block[18],
        rx_path=metadata_block[16],
        radar_mode=metadata_block[17] & 0x0F,
        inter_pulse_distance_in_ns=metadata_block[20],
        correlation_bit_shift=metadata_block[21],
        log_2_delta_L=metadata_block[22] & 0x3F,
        tx_pulse_shape_id=metadata_block[10],
        dc_freeze_enabled=bool((metadata_block[17] & 0x20) >> 5),
        single_pin=bool((metadata_block[17] & 0x10) >> 4),
        nsync_by_8=(le16(metadata_block[22:24]) & 0xFFC0) >> 6,
        peak_real_cir=le16(metadata_block[24:26]),
        num_pulses_used=le16(metadata_block[26:28]),
        cir_start_offset_fractional=le16(metadata_block[28:30]),
        cir_start_offset=le16(metadata_block[30:32]),
    )


def parse_taps(cir_data: bytes, bytes_per_tap: int) -> List[Tuple[int, int]]:
    taps: List[Tuple[int, int]] = []

    if bytes_per_tap == 4:
        for base in range(0, len(cir_data), 4):
            real = struct.unpack_from("<h", cir_data, base)[0]
            imag = struct.unpack_from("<h", cir_data, base + 2)[0]
            taps.append((real, imag))
    elif bytes_per_tap == 3:
        for base in range(0, len(cir_data), 3):
            real = struct.unpack_from("<h", cir_data, base)[0]
            imag = sign_extend_int8(cir_data[base + 2])
            taps.append((real, imag))
    else:
        raise ValueError(f"Unsupported tap width: {bytes_per_tap}")

    return taps


def parse_cir_udp_payload(payload: bytes) -> Dict[str, Any]:
    if len(payload) < 10:
        raise ValueError(f"Payload too short: {len(payload)} bytes")

    session_handle = le32(payload[0:4])
    status = payload[4]
    radar_data_type = payload[5]
    num_samples = le16(payload[6:8])
    taps_per_sample = payload[8]
    rfu = payload[9]

    sample_region = payload[10:]
    if num_samples == 0:
        raise ValueError("num_samples = 0")
    if len(sample_region) % num_samples != 0:
        raise ValueError(
            f"Remaining payload ({len(sample_region)}) not divisible by num_samples ({num_samples})"
        )

    block_size = len(sample_region) // num_samples
    if block_size < METADATA_BYTES:
        raise ValueError(f"Per-sample block too short: {block_size}")

    cir_data_bytes_per_block = block_size - METADATA_BYTES

    actual_cir_taps: Optional[int] = None
    bytes_per_tap: Optional[int] = None
    if taps_per_sample >= METADATA_TAPS:
        candidate_actual_taps = taps_per_sample - METADATA_TAPS
        if candidate_actual_taps > 0 and cir_data_bytes_per_block % candidate_actual_taps == 0:
            actual_cir_taps = candidate_actual_taps
            bytes_per_tap = cir_data_bytes_per_block // candidate_actual_taps

    if bytes_per_tap is None:
        for candidate in (4, 3):
            if cir_data_bytes_per_block % candidate == 0:
                bytes_per_tap = candidate
                actual_cir_taps = cir_data_bytes_per_block // candidate
                break

    if bytes_per_tap is None:
        raise ValueError(
            "Could not infer bytes_per_tap from payload shape: "
            f"block_size={block_size}, cir_data_bytes={cir_data_bytes_per_block}, "
            f"taps_per_sample={taps_per_sample}"
        )

    samples: List[Dict[str, Any]] = []
    for sample_index in range(num_samples):
        base = sample_index * block_size
        block = sample_region[base:base + block_size]
        metadata_block = block[:METADATA_BYTES]
        cir_data = block[METADATA_BYTES:]
        metadata = parse_metadata_block(
            metadata_block,
            session_handle=session_handle,
            status=status,
            radar_data_type=radar_data_type,
            num_samples=num_samples,
            taps_per_sample=taps_per_sample,
            rfu=rfu,
            sample_index=sample_index,
        )
        taps = parse_taps(cir_data, bytes_per_tap)
        samples.append(
            {
                "metadata": asdict(metadata),
                "num_cir_taps": len(taps),
                "taps": taps,
                "raw_cir_bytes": cir_data,
            }
        )

    return {
        "session_handle": session_handle,
        "status": status,
        "radar_data_type": radar_data_type,
        "num_samples": num_samples,
        "taps_per_sample": taps_per_sample,
        "rfu": rfu,
        "block_size": block_size,
        "bytes_per_tap": bytes_per_tap,
        "actual_cir_taps_per_block": actual_cir_taps,
        "samples": samples,
    }


def load_wireshark_hex_dump(path: str | Path) -> bytes:
    text = Path(path).read_text(encoding="utf-8")
    hex_bytes = re.findall(r"\b[0-9A-Fa-f]{2}\b", text)
    return bytes(int(x, 16) for x in hex_bytes)

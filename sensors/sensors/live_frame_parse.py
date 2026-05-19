"""Parse live UwbFrame messages into complex CIR arrays."""

from __future__ import annotations

import numpy as np

try:
    from sensors.sr250_protocol import parse_cir_udp_payload
except ImportError:
    from sr250_protocol import parse_cir_udp_payload


def message_to_path_taps(msg) -> np.ndarray | None:
    """Return (num_paths, num_taps) complex64 or None if not a CIR frame."""
    if int(msg.radar_data_type) != 0x00:
        return None

    parsed = parse_cir_udp_payload(bytes(msg.raw_payload))
    num_samples = int(parsed["num_samples"])
    if num_samples <= 0:
        return None

    taps_per_block = parsed["actual_cir_taps_per_block"]
    if taps_per_block is None:
        taps_per_block = len(parsed["samples"][0]["taps"]) if parsed["samples"] else 0
    if taps_per_block <= 0:
        return None

    taps = np.zeros((num_samples, taps_per_block), dtype=np.complex64)
    rx_paths = np.zeros((num_samples,), dtype=np.uint8)
    rx_antenna_ids = np.zeros((num_samples,), dtype=np.uint8)

    for sample_index, sample in enumerate(parsed["samples"]):
        meta = sample["metadata"]
        rx_paths[sample_index] = meta["rx_path"]
        rx_antenna_ids[sample_index] = meta["rx_antenna_id"]
        taps[sample_index, :] = np.asarray(
            [complex(real_part, imag_part) for real_part, imag_part in sample["taps"]],
            dtype=np.complex64,
        )

    # One UDP frame may bundle multiple RX paths; sort into stable path order.
    order = np.lexsort((rx_antenna_ids, rx_paths))
    return taps[order, :]

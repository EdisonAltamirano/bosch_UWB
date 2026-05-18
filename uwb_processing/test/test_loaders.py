from __future__ import annotations

from pathlib import Path

import numpy as np

from uwb_processing.loaders import _normalize_batched_frame_records, _stack_frame_records


def test_batched_notifications_are_reconstructed_into_logical_frames():
    tap_count = 3
    raw_records = [
        {
            "taps": np.asarray(
                [
                    [101 + 0j, 102 + 0j, 103 + 0j],  # counter 10, rx path 2
                    [201 + 0j, 202 + 0j, 203 + 0j],  # counter 10, rx path 1
                    [111 + 0j, 112 + 0j, 113 + 0j],  # counter 11, rx path 2
                    [211 + 0j, 212 + 0j, 213 + 0j],  # counter 11, rx path 1
                ],
                dtype=np.complex64,
            ),
            "timestamp_s": 1.0,
            "cir_counters": np.asarray([10, 10, 11, 11], dtype=np.uint32),
            "rx_paths": np.asarray([2, 1, 2, 1], dtype=np.uint8),
            "rx_timestamps": np.asarray([1000, 1000, 1100, 1100], dtype=np.uint32),
            "rx_antenna_ids": np.asarray([2, 1, 2, 1], dtype=np.uint8),
            "tx_antenna_ids": np.asarray([1, 1, 1, 1], dtype=np.uint8),
            "cir_start_offsets": np.asarray([0, 0, 0, 0], dtype=np.uint16),
            "bytes_per_tap": 4,
            "block_size": 512,
        },
        {
            "taps": np.asarray(
                [
                    [121 + 0j, 122 + 0j, 123 + 0j],  # counter 12, rx path 2
                    [221 + 0j, 222 + 0j, 223 + 0j],  # counter 12, rx path 1
                    [131 + 0j, 132 + 0j, 133 + 0j],  # counter 13, rx path 2
                    [231 + 0j, 232 + 0j, 233 + 0j],  # counter 13, rx path 1
                ],
                dtype=np.complex64,
            ),
            "timestamp_s": 1.2,
            "cir_counters": np.asarray([12, 12, 13, 13], dtype=np.uint32),
            "rx_paths": np.asarray([2, 1, 2, 1], dtype=np.uint8),
            "rx_timestamps": np.asarray([1200, 1200, 1300, 1300], dtype=np.uint32),
            "rx_antenna_ids": np.asarray([2, 1, 2, 1], dtype=np.uint8),
            "tx_antenna_ids": np.asarray([1, 1, 1, 1], dtype=np.uint8),
            "cir_start_offsets": np.asarray([0, 0, 0, 0], dtype=np.uint16),
            "bytes_per_tap": 4,
            "block_size": 512,
        },
    ]

    logical_records = _normalize_batched_frame_records(raw_records)

    assert len(logical_records) == 4
    assert all(record["taps"].shape == (2, tap_count) for record in logical_records)
    assert [int(record["cir_counter"]) for record in logical_records] == [10, 11, 12, 13]
    assert np.array_equal(logical_records[0]["rx_paths"], np.asarray([1, 2], dtype=np.uint8))
    assert np.allclose(
        [float(record["timestamp_s"]) for record in logical_records],
        [0.9, 1.0, 1.1, 1.2],
    )

    session = _stack_frame_records(Path("synthetic"), "synthetic", logical_records)

    assert session.frames.shape == (4, 2, tap_count)
    assert np.allclose(session.timestamps_s, [0.0, 0.1, 0.2, 0.3])
    assert np.array_equal(session.rx_antenna_ids, np.asarray([1, 2], dtype=np.uint8))
    assert np.allclose(session.frames[0, 0].real, [201, 202, 203])
    assert np.allclose(session.frames[0, 1].real, [101, 102, 103])

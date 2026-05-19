from __future__ import annotations

from pathlib import Path

import numpy as np

from uwb_processing.detection import detect_window
from uwb_processing.plotting import save_range_doppler_video
from uwb_processing.preprocessing import estimate_frame_rate_hz, preprocess_session
from uwb_processing.types import AnnotationWindow, DetectionConfig, RadarSession


def _make_session(moving: bool) -> RadarSession:
    rng = np.random.default_rng(7)
    frame_rate_hz = 20.0
    duration_s = 8.0
    timestamps_s = np.arange(0.0, duration_s, 1.0 / frame_rate_hz, dtype=np.float64)
    frame_count = timestamps_s.size
    path_count = 2
    tap_count = 96

    frames = (0.02 * rng.standard_normal((frame_count, path_count, tap_count))).astype(np.float32)
    frames = frames + 1j * (0.02 * rng.standard_normal((frame_count, path_count, tap_count))).astype(np.float32)

    static_tap = 12
    moving_tap = 36
    wall_return = 2.0 + 0j
    frames[:, :, static_tap] += wall_return
    if moving:
        phase = np.exp(1j * 2.0 * np.pi * 1.5 * timestamps_s)
        amplitude = 0.5 + 0.25 * np.sin(2.0 * np.pi * 0.35 * timestamps_s)
        frames[:, 1, moving_tap] += amplitude * phase

    return RadarSession(
        source_path=Path("synthetic"),
        source_kind="synthetic",
        frames=frames.astype(np.complex64),
        timestamps_s=timestamps_s,
        rx_antenna_ids=np.asarray([1, 2], dtype=np.uint8),
        tx_antenna_ids=np.asarray([1, 1], dtype=np.uint8),
        cir_start_offsets=np.asarray([0, 0], dtype=np.uint16),
        bytes_per_tap=4,
        block_size=tap_count,
        metadata={},
    )


def test_estimate_frame_rate_hz():
    timestamps = np.asarray([0.0, 0.05, 0.10, 0.15], dtype=np.float64)
    assert estimate_frame_rate_hz(timestamps) == 20.0


def test_presence_detection_distinguishes_motion():
    config = DetectionConfig(
        range_resolution_m=0.1,
        tap_spacing_m=0.1,
        default_range_gate_m=(2.5, 4.5),
        wall_clip_m=0.5,
        offline_rds_enabled=False,
        motion_threshold=2.0,
        doppler_window_s=2.0,
        microdoppler_window_s=1.0,
    )
    window = AnnotationWindow(label="walking_human", start_s=0.0, end_s=7.9, expected_range_m=(2.5, 4.5))
    artifact_root = Path(".")

    moving_session = preprocess_session(_make_session(moving=True), config=config, range_gate_m=window.expected_range_m)
    static_session = preprocess_session(_make_session(moving=False), config=config, range_gate_m=window.expected_range_m)

    moving_detection, _ = detect_window(moving_session, window, artifact_dir=artifact_root / "moving")
    static_detection, _ = detect_window(
        static_session,
        AnnotationWindow(label="empty", start_s=0.0, end_s=7.9, expected_range_m=(2.5, 4.5)),
        artifact_dir=artifact_root / "static",
    )

    assert moving_detection.predicted_present is True
    assert static_detection.predicted_present is False
    assert moving_detection.presence_score > static_detection.presence_score
    assert moving_detection.dominant_frequency_hz is not None


def test_save_range_doppler_video(tmp_path: Path):
    config = DetectionConfig(
        range_resolution_m=0.1,
        tap_spacing_m=0.1,
        default_range_gate_m=(2.5, 4.5),
        wall_clip_m=0.5,
        doppler_window_s=1.0,
    )
    preprocessed = preprocess_session(_make_session(moving=True), config=config)
    output_path = tmp_path / "range_doppler.mp4"
    save_range_doppler_video(preprocessed, output_path)
    assert output_path.exists()
    assert output_path.stat().st_size > 0

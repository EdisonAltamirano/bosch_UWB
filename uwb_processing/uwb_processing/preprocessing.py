from __future__ import annotations

import numpy as np
from scipy.signal import lfilter

from .types import DetectionConfig, PreprocessedSession, RadarSession


def estimate_frame_rate_hz(timestamps_s: np.ndarray) -> float:
    if timestamps_s.size < 2:
        return 1.0
    deltas = np.diff(timestamps_s)
    valid = deltas[deltas > 0]
    if valid.size == 0:
        return 1.0
    return float(1.0 / np.median(valid))


def compute_range_axis_m(session: RadarSession, config: DetectionConfig) -> np.ndarray:
    """Range per tap: r_k = (k - spillover_tap_0idx) * tap_spacing_m.

    For the NCJ29D6 the hardware delay is stripped by the UCI accumulator before the CIR
    window is delivered, so tap 0 ≈ 0 m and spillover_tap_0idx defaults to 0.  The
    cir_start_offset field in the metadata is the raw accumulator index (≈630 taps, ≈94 m)
    and must NOT be added here.
    """
    tap_count = session.frames.shape[2]
    k = np.arange(tap_count, dtype=np.float32)
    return (k - config.spillover_tap_0idx) * config.tap_spacing_m


def apply_rds_filter(frames: np.ndarray, frame_rate_hz: float, cutoff_hz: float = 0.2) -> np.ndarray:
    """First-order IIR high-pass (RDS) to remove thermal drift below cutoff_hz (AN-SCA-14453 §9.2)."""
    alpha = float(frame_rate_hz) / (float(frame_rate_hz) + 2.0 * np.pi * cutoff_hz)
    b = np.array([alpha, -alpha], dtype=np.float64)
    a = np.array([1.0, -alpha], dtype=np.float64)
    real_out = lfilter(b, a, frames.real.astype(np.float64), axis=0)
    imag_out = lfilter(b, a, frames.imag.astype(np.float64), axis=0)
    return (real_out + 1j * imag_out).astype(np.complex64)


def preprocess_session(
    session: RadarSession,
    config: DetectionConfig,
    range_gate_m: tuple[float, float] | None = None,
) -> PreprocessedSession:
    frame_rate_hz = estimate_frame_rate_hz(session.timestamps_s)
    range_axis_m = compute_range_axis_m(session, config)
    low_m, high_m = range_gate_m or config.default_range_gate_m

    # Step 1 — global mean clutter removal (AN-SCA-14453 §9.1)
    global_mean = session.frames.mean(axis=0, keepdims=True)
    clutter_removed_all = session.frames - global_mean

    # Step 2 — optional offline RDS IIR high-pass. Default off because the
    # current acquisition firmware already configures on-chip drift compensation.
    if config.offline_rds_enabled and config.rds_cutoff_hz > 0.0:
        filtered_all = apply_rds_filter(clutter_removed_all, frame_rate_hz, config.rds_cutoff_hz)
    else:
        filtered_all = clutter_removed_all

    wall_mask = range_axis_m >= config.wall_clip_m
    roi_mask = (range_axis_m >= low_m) & (range_axis_m <= high_m) & wall_mask
    if not np.any(roi_mask):
        roi_mask = wall_mask if np.any(wall_mask) else np.ones_like(range_axis_m, dtype=bool)

    # Step 3 — select path with highest power in ROI (AN-SCA-14453 )
    roi_power_by_path = np.mean(np.abs(filtered_all[:, :, roi_mask]) ** 2, axis=(0, 2))
    selected_path = int(np.argmax(roi_power_by_path))

    raw_magnitude = np.abs(session.frames[:, selected_path, :]).astype(np.float32)
    raw_magnitude[:, ~wall_mask] = 0.0

    clutter_hp = filtered_all[:, selected_path, :]  # complex64 (F, T)

    clutter_removed_mag = np.abs(clutter_hp).astype(np.float32)
    clutter_removed_mag[:, ~wall_mask] = 0.0

    # Step 4 — power (variance) per tap; dominant tap = argmax power (AN-SCA-14453)
    roi_tap_power = np.mean(np.abs(clutter_hp[:, roi_mask]) ** 2, axis=0)
    roi_tap_indices = np.flatnonzero(roi_mask)
    dominant_tap = int(roi_tap_indices[int(np.argmax(roi_tap_power))])

    variance_per_tap = np.mean(np.abs(clutter_hp) ** 2, axis=0).astype(np.float32)

    return PreprocessedSession(
        session=session,
        config=config,
        frame_rate_hz=frame_rate_hz,
        range_axis_m=range_axis_m,
        selected_path=selected_path,
        raw_magnitude=raw_magnitude,
        highpass_complex=clutter_hp.astype(np.complex64),
        clutter_removed=clutter_removed_mag,
        variance_per_tap=variance_per_tap,
        dominant_tap=dominant_tap,
    )

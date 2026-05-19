"""Shared range-Doppler math (offline and live viewers)."""

from __future__ import annotations

import numpy as np

_SPEED_OF_LIGHT_M_S = 3e8


def compute_range_axis_m(tap_count: int, tap_spacing_m: float, spillover_tap_0idx: int = 0) -> np.ndarray:
    k = np.arange(tap_count, dtype=np.float32)
    return (k - spillover_tap_0idx) * tap_spacing_m


def estimate_frame_rate_hz(timestamps_s: np.ndarray) -> float:
    if timestamps_s.size < 2:
        return 10.0
    deltas = np.diff(timestamps_s)
    valid = deltas[deltas > 0]
    if valid.size == 0:
        return 10.0
    return float(1.0 / np.median(valid))


def preprocess_buffer(
    frames: np.ndarray,
    range_axis_m: np.ndarray,
    range_gate_m: tuple[float, float],
    wall_clip_m: float,
) -> tuple[np.ndarray, int]:
    """Global-mean clutter removal and dominant-path selection on (N, P, T)."""
    global_mean = frames.mean(axis=0, keepdims=True)
    filtered = frames - global_mean

    low_m, high_m = range_gate_m
    wall_mask = range_axis_m >= wall_clip_m
    roi_mask = (range_axis_m >= low_m) & (range_axis_m <= high_m) & wall_mask
    if not np.any(roi_mask):
        roi_mask = wall_mask if np.any(wall_mask) else np.ones_like(range_axis_m, dtype=bool)

    roi_power_by_path = np.mean(np.abs(filtered[:, :, roi_mask]) ** 2, axis=(0, 2))
    selected_path = int(np.argmax(roi_power_by_path))
    return filtered[:, selected_path, :].astype(np.complex64), selected_path


def compute_range_doppler_map(
    signal: np.ndarray,
    frame_rate_hz: float,
    carrier_frequency_hz: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Range-Doppler from complex slow-time matrix (N, K)."""
    n_frames = signal.shape[0]
    taper = np.hanning(n_frames).reshape(-1, 1)
    rdm = np.fft.fftshift(np.fft.fft(signal * taper, axis=0), axes=0)
    rdm_db = (20.0 * np.log10(np.abs(rdm) + 1e-6)).astype(np.float32)
    doppler_hz = np.fft.fftshift(np.fft.fftfreq(n_frames, d=1.0 / frame_rate_hz))
    velocity_ms = (doppler_hz * _SPEED_OF_LIGHT_M_S / (2.0 * carrier_frequency_hz)).astype(np.float32)
    return rdm_db, velocity_ms

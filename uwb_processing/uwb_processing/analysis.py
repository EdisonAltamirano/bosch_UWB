from __future__ import annotations

import numpy as np
from scipy.signal import stft

from .types import PreprocessedSession, SpectrogramResult


def build_slow_time_signal(
    preprocessed: PreprocessedSession,
    frame_mask: np.ndarray,
    range_gate_m: tuple[float, float],
) -> tuple[np.ndarray, int]:
    """Extract complex IQ slow-time signal at the single power-dominant tap (AN-SCA-14453 §9.4)."""
    low_m, high_m = range_gate_m
    roi_mask = (preprocessed.range_axis_m >= low_m) & (preprocessed.range_axis_m <= high_m)
    if not np.any(roi_mask):
        roi_mask = preprocessed.range_axis_m >= preprocessed.config.wall_clip_m
    roi_indices = np.flatnonzero(roi_mask)
    window_signal = preprocessed.highpass_complex[frame_mask]  # (W, T) complex
    tap_power = np.mean(np.abs(window_signal[:, roi_mask]) ** 2, axis=0)
    dominant_local = int(np.argmax(tap_power))
    selected_tap = int(roi_indices[dominant_local])
    signal = window_signal[:, selected_tap]
    return signal.astype(np.complex64), selected_tap


def build_phase_signal(
    preprocessed: PreprocessedSession,
    frame_mask: np.ndarray,
    tap_index: int,
) -> np.ndarray:
    """Unwrapped phase of the dominant-tap CIR — input to micro-Doppler STFT (AN-SCA-14453 §9.5)."""
    signal_complex = preprocessed.highpass_complex[frame_mask][:, tap_index]
    return np.unwrap(np.angle(signal_complex.astype(np.complex128))).astype(np.float64)


def compute_spectrogram(
    signal: np.ndarray,
    frame_rate_hz: float,
    window_s: float,
    overlap: float,
    kind: str,
    selected_range_m: float,
    selected_tap: int,
    selected_path: int,
) -> SpectrogramResult:
    nperseg = max(16, int(round(frame_rate_hz * window_s)))
    nperseg = min(nperseg, signal.size)
    if nperseg < 4:
        frequencies = np.asarray([0.0], dtype=np.float32)
        times = np.asarray([0.0], dtype=np.float32)
        magnitude_db = np.asarray([[0.0]], dtype=np.float32)
        return SpectrogramResult(kind, frequencies, times, magnitude_db, selected_range_m, selected_tap, selected_path)

    noverlap = min(nperseg - 1, max(0, int(round(nperseg * overlap))))
    frequencies, times, zxx = stft(
        signal,
        fs=frame_rate_hz,
        nperseg=nperseg,
        noverlap=noverlap,
        return_onesided=False,
        boundary=None,
        padded=False,
    )
    frequencies = np.fft.fftshift(frequencies)
    zxx = np.fft.fftshift(zxx, axes=0)
    magnitude_db = (20.0 * np.log10(np.abs(zxx) + 1e-6)).astype(np.float32)
    return SpectrogramResult(
        kind=kind,
        frequencies_hz=frequencies.astype(np.float32),
        times_s=times.astype(np.float32),
        magnitude_db=magnitude_db,
        selected_range_m=selected_range_m,
        selected_tap=selected_tap,
        selected_path=selected_path,
    )


def dominant_frequency_hz(spectrogram: SpectrogramResult, zero_doppler_hz: float) -> float | None:
    if spectrogram.magnitude_db.size == 0:
        return None
    valid = np.abs(spectrogram.frequencies_hz) >= zero_doppler_hz
    if not np.any(valid):
        return None
    band_power = spectrogram.magnitude_db[valid].mean(axis=1)
    return float(spectrogram.frequencies_hz[valid][int(np.argmax(band_power))])

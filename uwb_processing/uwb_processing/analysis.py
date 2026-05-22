from __future__ import annotations

import numpy as np
from scipy.signal import stft

from .types import PreprocessedSession, SpectrogramResult, Track


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


def build_track_slow_time_signal(
    preprocessed: PreprocessedSession,
    track: Track,
    tap_gate_m: float = 0.30,
) -> tuple[np.ndarray, float]:
    """Extract complex IQ slow-time signal aligned to a confirmed track's range trajectory.

    For each frame in the track's history, finds the tap of maximum CIR magnitude
    within ±tap_gate_m of the Kalman-estimated range.  This follows the target as
    it moves — unlike the session-level dominant_tap which is a single static choice.

    Returns (signal complex64 shape (N,), mean_range_m float).
    """
    timestamps = preprocessed.session.timestamps_s
    cir = preprocessed.highpass_complex            # (N_frames, N_taps)
    tap_spacing = preprocessed.config.tap_spacing_m
    gate_taps = max(1, int(round(tap_gate_m / tap_spacing)))
    n_taps = cir.shape[1]

    signal_vals: list[complex] = []
    for t_hist, r_hist in zip(track.history_t, track.history_m):
        fi = int(np.argmin(np.abs(timestamps - t_hist)))
        center_tap = int(np.round(r_hist / tap_spacing))
        center_tap = int(np.clip(center_tap, 0, n_taps - 1))
        lo = max(0, center_tap - gate_taps)
        hi = min(n_taps - 1, center_tap + gate_taps)
        local_mag = np.abs(cir[fi, lo : hi + 1])
        best_tap = lo + int(np.argmax(local_mag))
        signal_vals.append(complex(cir[fi, best_tap]))

    if not signal_vals:
        return np.array([], dtype=np.complex64), 0.0

    signal = np.array(signal_vals, dtype=np.complex64)
    mean_range_m = float(np.mean(track.history_m))
    return signal, mean_range_m


def dominant_frequency_hz(spectrogram: SpectrogramResult, zero_doppler_hz: float) -> float | None:
    if spectrogram.magnitude_db.size == 0:
        return None
    valid = np.abs(spectrogram.frequencies_hz) >= zero_doppler_hz
    if not np.any(valid):
        return None
    band_power = spectrogram.magnitude_db[valid].mean(axis=1)
    return float(spectrogram.frequencies_hz[valid][int(np.argmax(band_power))])
